/*
 * Copyright (c) 2019 Pilz GmbH & Co. KG
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.

 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <prbt_hardware_support/brake_test_executor.h>

#include <sstream>

#include <XmlRpcValue.h>

#include <prbt_hardware_support/brake_test_utils.h>
#include <prbt_hardware_support/brake_test_executor_exception.h>

namespace prbt_hardware_support
{

BrakeTestExecutor::BrakeTestExecutor(ros::NodeHandle& nh)
  :nh_(nh)
{
  brake_test_srv_ = nh_.advertiseService("prbt/execute_braketest",
                                         &BrakeTestExecutor::executeBraketest,
                                         this);

  canopen_srv_client_ = nh_.serviceClient<BrakeTest>("prbt/driver/get_object");
}

ros::Duration BrakeTestExecutor::getBrakeTestDuration(const std::string& joint_name)
{
  canopen_chain_node::GetObject srv;
  srv.request.node = joint_name;
  srv.request.object = "2060sub1";
  srv.request.cached = false;

  if (!canopen_srv_client_.call(srv))
  {
    throw BrakeTestExecutorException("CANopen service to request brake test duration failed"); //=> service response invalid
  }

  if (!srv.response.success)
  {
    throw BrakeTestExecutorException(srv.response.message);
  }

  return ros::Duration(  std::stoi(srv.response.value)/1000  );
}

void BrakeTestExecutor::triggerBrakeTestForJoint(const std::string& joint_name)
{
  canopen_chain_node::SetObject srv;
  srv.request.node = joint_name;
  srv.request.object = "2060sub2";
  srv.request.value = 1; // Demand brake test
  srv.request.cached = false;

  if (!canopen_srv_client_.call(srv))
  {
    throw BrakeTestExecutorException("CANopen service for brake test execution failed"); //=> service response invalid
  }

  if (!srv.response.success)
  {
    throw BrakeTestExecutorException(srv.response.message);
  }
}

BrakeTestExecutor::BrakeTestStatus BrakeTestExecutor::getBrakeTestStatusForJoint(const std::string& joint_name)
{
  canopen_chain_node::GetObject srv;
  srv.request.node = joint_name;
  srv.request.object = "2060sub3";
  srv.request.cached = false;

  if (!canopen_srv_client_.call(srv))
  {
    throw BrakeTestExecutorException("CANopen service to request brake test status failed"); //=> service response invalid
  }

  // TODO: Ignore "srv.response.success" because of its redundancy with "srv.response.value"?

  BrakeTestStatus status;
  status.first = static_cast<int8_t>(std::stoi(srv.response.value));
  status.second = srv.response.message;
  return status;
}

bool BrakeTestExecutor::executeBraketest(BrakeTest::Request&,
                                         BrakeTest::Response& response)
{
  if (BrakeTestUtils::detectRobotMotion())
  {
    response.msg = "Robot is moving, cannot perform brake test";
    response.result = BrakeTest::Response::ROBOT_MOTION_DETECTED;
    return true;
  }

  XmlRpc::XmlRpcValue rpc;
  if ( !nh_.getParam("/prbt/nodes", rpc) )
  {
    response.msg = "Could not read joint names";
    response.result = BrakeTest::Response::JOINT_NAMES_NOT_FOUND;
    return true;
  }

  for(auto rpci = rpc.begin(); rpci != rpc.end(); ++rpci)
  {
    std::string joint_name {rpci->first.c_str()};
    ROS_INFO_STREAM("Perform brake test for joint \"" << joint_name << "\"...");
    try
    {
      ros::Duration brake_test_duration(getBrakeTestDuration(joint_name));
      triggerBrakeTestForJoint(joint_name);
      brake_test_duration.sleep();
      BrakeTestStatus status {getBrakeTestStatusForJoint(joint_name)};
      response.result = status.first;
      response.msg = status.second;
    }
    catch (const BrakeTestExecutorException& ex)
    {
      response.msg = ex.what();
      response.result = BrakeTest::Response::FAILURE;
      return true;
    }
  }

  return true;
}

}
