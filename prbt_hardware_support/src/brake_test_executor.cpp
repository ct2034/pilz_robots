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
  brake_test_srv_ = nh_.advertiseService("execute_braketest",
                                         &BrakeTestExecutor::executeBraketest,
                                         this);

  canopen_srv_get_client_ = nh_general_.serviceClient<canopen_chain_node::GetObject>("driver/get_object");
  canopen_srv_set_client_ = nh_general_.serviceClient<canopen_chain_node::SetObject>("driver/set_object");

  canopen_srv_get_client_.waitForExistence();
  canopen_srv_set_client_.waitForExistence();
}

ros::Duration BrakeTestExecutor::getBrakeTestDuration(const std::string& joint_name)
{
  canopen_chain_node::GetObject srv;
  srv.request.node = joint_name;
  srv.request.object = "2060sub1";
  srv.request.cached = false;

  ROS_INFO_STREAM("Call \"brake test duration\" service for \"" << joint_name << "\"");
  if (!canopen_srv_get_client_.call(srv))
  {
    throw BrakeTestExecutorException("CANopen service to request brake test duration failed",
                                     BrakeTest::Response::FAILED_TO_DETERMINE_DURATION); //=> service response invalid
  }

  if (!srv.response.success)
  {
    throw BrakeTestExecutorException(srv.response.message,
                                     BrakeTest::Response::FAILED_TO_DETERMINE_DURATION);
  }

  ROS_INFO_STREAM("Brake test duration for joint \"" << joint_name << "\" is: " << srv.response.value << "ms");
  return ros::Duration(  std::stoi(srv.response.value)/1000, 0  );
}

void BrakeTestExecutor::triggerBrakeTestForJoint(const std::string& joint_name)
{
  canopen_chain_node::SetObject srv;
  srv.request.node = joint_name;
  srv.request.object = "2060sub2";
  srv.request.value = "1"; // Demand brake test
  srv.request.cached = false;

  ROS_INFO_STREAM("Call \"trigger brake test\" service for \"" << joint_name << "\"");
  if (!canopen_srv_set_client_.call(srv))
  {
    throw BrakeTestExecutorException("CANopen service for brake test execution failed",
                                     BrakeTest::Response::TRIGGERING_OF_BRAKE_TEST_FAILED); //=> service response invalid
  }

  if (!srv.response.success)
  {
    throw BrakeTestExecutorException(srv.response.message,
                                     BrakeTest::Response::TRIGGERING_OF_BRAKE_TEST_FAILED);
  }
}

BrakeTestExecutor::BrakeTestStatus BrakeTestExecutor::getBrakeTestStatusForJoint(const std::string& joint_name)
{
  canopen_chain_node::GetObject srv;
  srv.request.node = joint_name;
  srv.request.object = "2060sub3";
  srv.request.cached = false;

  ROS_INFO_STREAM("Call \"get status brake test\" service for \"" << joint_name << "\"");
  if (!canopen_srv_get_client_.call(srv))
  {
    throw BrakeTestExecutorException("CANopen service to request brake test status failed",
                                     BrakeTest::Response::FAILED_TO_GET_STATUS); //=> service response invalid
  }

  if (!srv.response.success)
  {
    ROS_WARN_STREAM("Service returned: success == false");
    throw BrakeTestExecutorException("Reading of CANopen to determine brake test status failed",
                                     BrakeTest::Response::FAILED_TO_GET_STATUS);
  }

  BrakeTestStatus status;
  status.first = static_cast<int8_t>(srv.response.value.data()[0]);
  status.second = srv.response.message;
  return status;
}

void BrakeTestExecutor::checkBrakeTestResultForJoint(const std::string& joint_name)
{
  BrakeTestStatus status {getBrakeTestStatusForJoint(joint_name)};
  if (status.first != BrakeTest::Response::SUCCESS)
  {
    ROS_ERROR("Brake test for %s failed (Status: %d)", joint_name.c_str(), status.first);
    throw BrakeTestExecutorException(status.second, status.first);
  }
}

bool BrakeTestExecutor::executeBraketest(BrakeTest::Request&,
                                         BrakeTest::Response& response)
{
  if (BrakeTestUtils::detectRobotMotion())
  {
    response.msg = "Robot is moving, cannot perform brake tes";
    response.result = BrakeTest::Response::ROBOT_MOTION_DETECTED;
    return true;
  }

  XmlRpc::XmlRpcValue rpc;
  if ( !nh_.getParam("/prbt/driver/nodes", rpc) )
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

      checkBrakeTestResultForJoint(joint_name);
    }
    catch (const BrakeTestExecutorException& ex)
    {
      ROS_ERROR_STREAM("Brake test failed: " << ex.what());
      response.msg = ex.what();
      response.result = ex.getErrorValue();
      return true;
    }
  }

  response.msg = "";
  response.result = BrakeTest::Response::SUCCESS;
  return true;
}

}
