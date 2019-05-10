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

#include <functional>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <XmlRpcValue.h>

#include <ros/ros.h>
#include <canopen_chain_node/GetObject.h>
#include <canopen_chain_node/SetObject.h>

#include <prbt_hardware_support/brake_test_executor.h>
#include <prbt_hardware_support/BrakeTest.h>
#include <prbt_hardware_support/joint_states_publisher_mock.h>

namespace brake_test_executor_test
{

using namespace prbt_hardware_support;
using namespace testing;

using canopen_chain_node::GetObjectRequest;
using canopen_chain_node::GetObjectResponse;
using canopen_chain_node::SetObjectRequest;
using canopen_chain_node::SetObjectResponse;

static const std::string CANOPEN_GETOBJECT_SERVICE_NAME{"driver/get_object"};
static const std::string CANOPEN_SETOBJECT_SERVICE_NAME{"driver/set_object"};
static const std::string BRAKE_TEST_SERVICE_NAME{"execute_braketest"};

static const std::string BRAKE_TEST_DURATION_OBJECT_INDEX{"2060sub1"};
static const std::string START_BRAKE_TEST_OBJECT_INDEX{"2060sub2"};
static const std::string BRAKE_TEST_STATUS_OBJECT_INDEX{"2060sub3"};

static const std::string NODE_NAMES_PARAMETER_NAME{"/prbt/driver/nodes"};
static const std::string NODE_NAMES_PREFIX{"prbt_joint_"};
static constexpr int NODE_COUNT{6};
static const std::vector<size_t> NODE_TEST_SET{{0, 2, 5}};

class CANOpenChainNodeMock
{
public:
  /**
   * @brief Advertise get_object and set_object services for CANOpen objects and call setDefaultActions().
   */
  CANOpenChainNodeMock();

  /**
   * @brief Set default actions on all expected service calls.
   *
   * Does not set any expectations.
   */
  void setDefaultActions();

  /**
   * @brief Set expectations on all mock methods, that can be fullfilled in any case.
   *
   * Allows any number of calls on get_obj() and set_obj.
   */
  void expectAnything();

  /**
   * @brief Un-advertise services.
   */
  void shutdown();

public:
  MOCK_METHOD2(get_obj, bool(canopen_chain_node::GetObjectRequest &, canopen_chain_node::GetObjectResponse &));
  MOCK_METHOD2(set_obj, bool(canopen_chain_node::SetObjectRequest &, canopen_chain_node::SetObjectResponse &));

private:
  ros::NodeHandle nh_;
  ros::ServiceServer get_obj_serv_;
  ros::ServiceServer set_obj_serv_;
};

CANOpenChainNodeMock::CANOpenChainNodeMock()
{
  get_obj_serv_ = nh_.advertiseService(CANOPEN_GETOBJECT_SERVICE_NAME, &CANOpenChainNodeMock::get_obj, this);
  set_obj_serv_ = nh_.advertiseService(CANOPEN_SETOBJECT_SERVICE_NAME, &CANOpenChainNodeMock::set_obj, this);

  setDefaultActions();
}

void CANOpenChainNodeMock::setDefaultActions()
{
  GetObjectResponse duration_resp;
  duration_resp.success = true;
  duration_resp.value = "1";

  SetObjectResponse start_resp;
  start_resp.success = true;

  GetObjectResponse status_resp;
  status_resp.success = true;
  status_resp.value = "\x02";

  // Set response for service calls getting the brake_test_duration object
  ON_CALL(*this, get_obj(Field(&GetObjectRequest::object, BRAKE_TEST_DURATION_OBJECT_INDEX), _))
      .WillByDefault(DoAll(SetArgReferee<1>(duration_resp), Return(true)));

  // Set response for service calls setting the start_brake_test object
  ON_CALL(*this, set_obj(Field(&SetObjectRequest::object, START_BRAKE_TEST_OBJECT_INDEX), _))
      .WillByDefault(DoAll(SetArgReferee<1>(start_resp), Return(true)));

  // Set response for service calls getting the brake_test_status object
  ON_CALL(*this, get_obj(Field(&GetObjectRequest::object, BRAKE_TEST_STATUS_OBJECT_INDEX), _))
      .WillByDefault(DoAll(SetArgReferee<1>(status_resp), Return(true)));
}

void CANOpenChainNodeMock::expectAnything()
{
  EXPECT_CALL(*this, get_obj(_, _))
      .Times(AnyNumber());
  EXPECT_CALL(*this, set_obj(_, _))
      .Times(AnyNumber());
}

void CANOpenChainNodeMock::shutdown()
{
  get_obj_serv_.shutdown();
  set_obj_serv_.shutdown();
}

/**
 * @brief Construct CANOpenChainNodeMock and BrakeTestExecutor objects and service client for the BrakeTest service.
 */
class BrakeTestExecutorTest : public Test
{
public:
  BrakeTestExecutorTest();
  void SetUp() override;

protected:
  ros::NodeHandle nh_;
  ros::ServiceClient brake_test_srv_client_;
  std::unique_ptr<BrakeTestExecutor> brake_test_executor_;
  CANOpenChainNodeMock canopen_chain_node_;
};

BrakeTestExecutorTest::BrakeTestExecutorTest()
{
  brake_test_srv_client_ = nh_.serviceClient<BrakeTest>(BRAKE_TEST_SERVICE_NAME);
  brake_test_executor_.reset(new BrakeTestExecutor(nh_));
}

void BrakeTestExecutorTest::SetUp()
{
  ASSERT_TRUE(brake_test_srv_client_.exists()) << "Brake test service not available.";
}

/**
 * @brief Test execution of brake tests.
 *
 * Test Sequence:
 *  1. Set expectations on CANOpen mock object.
 *  2. Publish fixed joint states.
 *  3. Call brake test service.
 *
 * Expected Results:
 *  1. -
 *  2. -
 *  3. Brake tests are executed successfully. For all nodes there is:
 *     - at least one read request on the brake_test_duration object,
 *     - exactly one write request,
 *     - at least one read request on the brake_test_status object.
 */
TEST_F(BrakeTestExecutorTest, testBrakeTestService)
{
  /**********
   * Step 1 *
   **********/
  for (int i = 0; i < NODE_COUNT; ++i)
  {
    std::string node{NODE_NAMES_PREFIX + std::to_string(i + 1)};

    EXPECT_CALL(canopen_chain_node_, get_obj(AllOf(Field(&GetObjectRequest::node, node),
                                                   Field(&GetObjectRequest::object, BRAKE_TEST_DURATION_OBJECT_INDEX)),
                                             _))
        .Times(AtLeast(1));

    EXPECT_CALL(canopen_chain_node_, set_obj(Field(&SetObjectRequest::node, node), _))
        .Times(1);

    EXPECT_CALL(canopen_chain_node_, get_obj(AllOf(Field(&GetObjectRequest::node, node),
                                                   Field(&GetObjectRequest::object, BRAKE_TEST_STATUS_OBJECT_INDEX)),
                                             _))
        .Times(AtLeast(1));
  }

  /**********
   * Step 2 *
   **********/
  JointStatesPublisherMock joint_states_pub;
  joint_states_pub.startAsync();

  /**********
   * Step 3 *
   **********/
  BrakeTest srv;
  EXPECT_TRUE(brake_test_srv_client_.call(srv)) << "Failed to call brake test service.";
  EXPECT_EQ(BrakeTestResponse::SUCCESS, srv.response.result) << "Brake tests failed unexpectedly.";
}

/**
 * @brief Test execution of brake tests without CAN services.
 *
 * Test Sequence:
 *  1. Shutdown CAN service mocks.
 *  2. Publish fixed joint states.
 *  3. Call brake test service.
 *
 * Expected Results:
 *  1. -
 *  2. -
 *  3. Brake test service responds with correct error case.
 */
TEST_F(BrakeTestExecutorTest, testBrakeTestServiceWithoutCANServices)
{
  /**********
   * Step 1 *
   **********/
  canopen_chain_node_.shutdown();

  /**********
   * Step 2 *
   **********/
  JointStatesPublisherMock joint_states_pub;
  joint_states_pub.startAsync();

  /**********
   * Step 3 *
   **********/
  BrakeTest srv;
  EXPECT_TRUE(brake_test_srv_client_.call(srv)) << "Failed to call brake test service.";
  EXPECT_TRUE(srv.response.result == BrakeTestResponse::TRIGGERING_OF_BRAKE_TEST_FAILED ||
              srv.response.result == BrakeTestResponse::FAILED_TO_DETERMINE_DURATION ||
              srv.response.result == BrakeTestResponse::FAILED_TO_GET_STATUS);
}

/**
 * @brief Test execution of brake tests without node names on the parameter server.
 *
 * Test Sequence:
 *  1. Delete node names parameter.
 *  2. Publish fixed joint states.
 *  3. Call brake test service.
 *  4. Restore node names parameter.
 *
 * Expected Results:
 *  1. -
 *  2. -
 *  3. Brake test service responds with correct error case.
 *  4. -
 */
TEST_F(BrakeTestExecutorTest, testBrakeTestServiceWithoutNodeParameters)
{
  /**********
   * Step 1 *
   **********/
  XmlRpc::XmlRpcValue rpc;
  ASSERT_TRUE(nh_.getParam(NODE_NAMES_PARAMETER_NAME, rpc));
  nh_.deleteParam(NODE_NAMES_PARAMETER_NAME);

  /**********
   * Step 2 *
   **********/
  JointStatesPublisherMock joint_states_pub;
  joint_states_pub.startAsync();

  /**********
   * Step 3 *
   **********/
  BrakeTest srv;
  EXPECT_TRUE(brake_test_srv_client_.call(srv)) << "Failed to call brake test service.";
  EXPECT_EQ(BrakeTestResponse::JOINT_NAMES_NOT_FOUND, srv.response.result);

  nh_.setParam(NODE_NAMES_PARAMETER_NAME, rpc);
}

/**
 * @brief Test execution of brake tests with moving robot.
 *
 * Test Sequence:
 *  1. Set expectations on CANOpen mock object.
 *  2. Publish changing joint states.
 *  3. Call brake test service.
 *
 * Expected Results:
 *  1. Brake test service responds with correct error case. No write requests on canopen objects.
 */
TEST_F(BrakeTestExecutorTest, testBrakeTestServiceWithRobotMotion)
{
  /**********
   * Step 1 *
   **********/
  EXPECT_CALL(canopen_chain_node_, set_obj(_, _)).Times(0);

  /**********
   * Step 2 *
   **********/
  JointStatesPublisherMock joint_states_pub;
  joint_states_pub.startAsync(true);

  /**********
   * Step 3 *
   **********/
  BrakeTest srv;
  EXPECT_TRUE(brake_test_srv_client_.call(srv)) << "Failed to call brake test service.";
  EXPECT_EQ(BrakeTestResponse::ROBOT_MOTION_DETECTED, srv.response.result);
}

/**
 * @brief Test execution of brake tests when the service call getting the brake_test_duration object fails.
 *
 * This is repeated for the first and the last node and one in between.
 *
 * Test Sequence:
 *  1. Set expectations on CANOpen mock object. Let service call getting the brake_test_duration object fail for the
 *     selected node.
 *  2. Publish fixed joint states.
 *  3. Call brake test service.
 *  4. Verify and clear expectations.
 *
 * Expected Results:
 *  1. -
 *  2. -
 *  3. The brake test service responds with correct error case.
 *  4. There is:
 *     - at least one read request on the brake_test_duration object for the selected node,
 *     - no write request for the selected node.
 */
TEST_F(BrakeTestExecutorTest, testGetBrakeTestDurationServiceCallFailure)
{
  for (size_t node_index : NODE_TEST_SET)
  {
    std::string node{NODE_NAMES_PREFIX + std::to_string(node_index + 1)};

    /**********
     * Step 1 *
     **********/
    // Avoid unmatched expectations
    canopen_chain_node_.expectAnything();

    EXPECT_CALL(canopen_chain_node_, get_obj(AllOf(Field(&GetObjectRequest::node, node),
                                                   Field(&GetObjectRequest::object, BRAKE_TEST_DURATION_OBJECT_INDEX)),
                                             _))
        .Times(AtLeast(1))
        .WillRepeatedly(Return(false));

    EXPECT_CALL(canopen_chain_node_, set_obj(AllOf(Field(&SetObjectRequest::node, node),
                                                   Field(&SetObjectRequest::object, START_BRAKE_TEST_OBJECT_INDEX)), _))
        .Times(0);

    /**********
     * Step 2 *
     **********/
    JointStatesPublisherMock joint_states_pub;
    joint_states_pub.startAsync();

    /**********
     * Step 3 *
     **********/
    BrakeTest srv;
    EXPECT_TRUE(brake_test_srv_client_.call(srv)) << "Failed to call brake test service.";
    EXPECT_EQ(BrakeTestResponse::FAILED_TO_DETERMINE_DURATION, srv.response.result);

    /**********
     * Step 4 *
     **********/
    ASSERT_TRUE(Mock::VerifyAndClearExpectations(&canopen_chain_node_));
  }
}

/**
 * @brief Test execution of brake tests when the service call getting the brake_test_duration object responds an error.
 *
 * This is repeated for the first and the last node and one in between.
 *
 * Test Sequence:
 *  1. Set expectations on CANOpen mock object. Let service call getting the brake_test_duration object respond an error
 *     for the selected node.
 *  2. Publish fixed joint states.
 *  3. Call brake test service.
 *  4. Verify and clear expectations.
 *
 * Expected Results:
 *  1. -
 *  2. -
 *  3. The brake test service responds with correct error case.
 *  4. There is:
 *     - at least one read request on the brake_test_duration object for the selected node,
 *     - no write request for the selected node.
 */
TEST_F(BrakeTestExecutorTest, testGetBrakeTestDurationServiceResponseFailure)
{
  GetObjectResponse duration_resp;
  duration_resp.success = false;

  for (size_t node_index : NODE_TEST_SET)
  {
    std::string node{NODE_NAMES_PREFIX + std::to_string(node_index + 1)};

    /**********
     * Step 1 *
     **********/
    // Avoid unmatched expectations
    canopen_chain_node_.expectAnything();

    EXPECT_CALL(canopen_chain_node_, get_obj(AllOf(Field(&GetObjectRequest::node, node),
                                                   Field(&GetObjectRequest::object, BRAKE_TEST_DURATION_OBJECT_INDEX)),
                                             _))
        .Times(AtLeast(1))
        .WillRepeatedly(DoAll(SetArgReferee<1>(duration_resp), Return(true)));

    EXPECT_CALL(canopen_chain_node_, set_obj(AllOf(Field(&SetObjectRequest::node, node),
                                                   Field(&SetObjectRequest::object, START_BRAKE_TEST_OBJECT_INDEX)), _))
        .Times(0);

    /**********
     * Step 2 *
     **********/
    JointStatesPublisherMock joint_states_pub;
    joint_states_pub.startAsync();

    /**********
     * Step 3 *
     **********/
    BrakeTest srv;
    EXPECT_TRUE(brake_test_srv_client_.call(srv)) << "Failed to call brake test service.";
    EXPECT_EQ(BrakeTestResponse::FAILED_TO_DETERMINE_DURATION, srv.response.result);

    /**********
     * Step 4 *
     **********/
    ASSERT_TRUE(Mock::VerifyAndClearExpectations(&canopen_chain_node_));
  }
}

/**
 * @brief Test execution of brake tests when the service call setting the start_brake_test object fails.
 *
 * This is repeated for the first and the last node and one in between.
 *
 * Test Sequence:
 *  1. Set expectations on CANOpen mock object. Let the service call getting the start_brake_test object fail for the
 *     selected node.
 *  2. Publish fixed joint states.
 *  3. Call brake test service.
 *  4. Verify and clear expectations.
 *
 * Expected Results:
 *  1. -
 *  2. -
 *  3. The brake test service responds with correct error case.
 *  4. There is:
 *     - at least one read request on brake_test_duration object for the selected node,
 *     - exactly one write request for the selected node.
 */
TEST_F(BrakeTestExecutorTest, testStartBrakeTestServiceCallFailure)
{
  for (size_t node_index : NODE_TEST_SET)
  {
    std::string node{NODE_NAMES_PREFIX + std::to_string(node_index + 1)};

    /**********
     * Step 1 *
     **********/
    // Avoid unmatched expectations
    canopen_chain_node_.expectAnything();

    EXPECT_CALL(canopen_chain_node_, get_obj(AllOf(Field(&GetObjectRequest::node, node),
                                                   Field(&GetObjectRequest::object, BRAKE_TEST_DURATION_OBJECT_INDEX)),
                                             _))
        .Times(AtLeast(1));

    EXPECT_CALL(canopen_chain_node_, set_obj(AllOf(Field(&SetObjectRequest::node, node),
                                                   Field(&SetObjectRequest::object, START_BRAKE_TEST_OBJECT_INDEX)), _))
        .WillOnce(Return(false));

    /**********
     * Step 2 *
     **********/
    JointStatesPublisherMock joint_states_pub;
    joint_states_pub.startAsync();

    /**********
     * Step 3 *
     **********/
    BrakeTest srv;
    EXPECT_TRUE(brake_test_srv_client_.call(srv)) << "Failed to call brake test service.";
    EXPECT_EQ(BrakeTestResponse::TRIGGERING_OF_BRAKE_TEST_FAILED, srv.response.result);

    /**********
     * Step 4 *
     **********/
    Mock::VerifyAndClearExpectations(&canopen_chain_node_);
  }
}

/**
 * @brief Test execution of brake tests when the service call setting the start_brake_test object responds an error.
 *
 * This is repeated for the first and the last node and one in between.
 *
 * Test Sequence:
 *  1. Set expectations on CANOpen mock object. Let the service call getting the start_brake_test object respond an error
 *     for the selected node.
 *  2. Publish fixed joint states.
 *  3. Call brake test service.
 *  4. Verify and clear expectations.
 *
 * Expected Results:
 *  1. -
 *  2. -
 *  3. The brake test service responds with correct error case.
 *  4. There is:
 *     - at least one read request on brake_test_duration object for the selected node,
 *     - exactly one write request for the selected node.
 */
TEST_F(BrakeTestExecutorTest, testStartBrakeTestServiceResponseFailure)
{
  SetObjectResponse start_resp;
  start_resp.success = false;

  for (size_t node_index : NODE_TEST_SET)
  {
    std::string node{NODE_NAMES_PREFIX + std::to_string(node_index + 1)};

    /**********
     * Step 1 *
     **********/
    // Avoid unmatched expectations
    canopen_chain_node_.expectAnything();

    EXPECT_CALL(canopen_chain_node_, get_obj(AllOf(Field(&GetObjectRequest::node, node),
                                                   Field(&GetObjectRequest::object, BRAKE_TEST_DURATION_OBJECT_INDEX)),
                                             _))
        .Times(AtLeast(1));

    EXPECT_CALL(canopen_chain_node_, set_obj(AllOf(Field(&SetObjectRequest::node, node),
                                                   Field(&SetObjectRequest::object, START_BRAKE_TEST_OBJECT_INDEX)), _))
        .WillOnce(DoAll(SetArgReferee<1>(start_resp), Return(true)));

    /**********
     * Step 2 *
     **********/
    JointStatesPublisherMock joint_states_pub;
    joint_states_pub.startAsync();

    /**********
     * Step 3 *
     **********/
    BrakeTest srv;
    EXPECT_TRUE(brake_test_srv_client_.call(srv)) << "Failed to call brake test service.";
    EXPECT_EQ(BrakeTestResponse::TRIGGERING_OF_BRAKE_TEST_FAILED, srv.response.result);

    /**********
     * Step 4 *
     **********/
    Mock::VerifyAndClearExpectations(&canopen_chain_node_);
  }
}

/**
 * @brief Test execution of brake tests when the service call getting the brake_test_status object fails.
 *
 * This is repeated for the first and the last node and one in between.
 *
 * Test Sequence:
 *  1. Set expectations on CANOpen mock object. Let all services respond success, but let the service call getting the
 *     brake_test_status object fail for a selected node.
 *  2. Publish fixed joint states.
 *  3. Call brake test service.
 *  4. Verify and clear expectations.
 *
 * Expected Results:
 *  1. -
 *  2. -
 *  3. The brake test service responds with correct error case.
 *  4. There is at least one read request on brake_test_status object for the selected node.
 */
TEST_F(BrakeTestExecutorTest, testBrakeTestStatusServiceCallFailure)
{
  for (size_t node_index : NODE_TEST_SET)
  {
    std::string node{NODE_NAMES_PREFIX + std::to_string(node_index + 1)};

    /**********
     * Step 1 *
     **********/
    // Avoid unmatched expectations
    canopen_chain_node_.expectAnything();

    EXPECT_CALL(canopen_chain_node_, get_obj(AllOf(Field(&GetObjectRequest::node, node),
                                                   Field(&GetObjectRequest::object, BRAKE_TEST_STATUS_OBJECT_INDEX)),
                                             _))
        .Times(AtLeast(1))
        .WillRepeatedly(Return(false));

    /**********
     * Step 2 *
     **********/
    JointStatesPublisherMock joint_states_pub;
    joint_states_pub.startAsync();

    /**********
     * Step 3 *
     **********/
    BrakeTest srv;
    EXPECT_TRUE(brake_test_srv_client_.call(srv)) << "Failed to call brake test service.";
    EXPECT_EQ(BrakeTestResponse::FAILED_TO_GET_STATUS, srv.response.result);

    /**********
     * Step 4 *
     **********/
    Mock::VerifyAndClearExpectations(&canopen_chain_node_);
  }
}

/**
 * @brief Test execution of brake tests when the service call getting the brake_test_status object responds an error.
 *
 * This is repeated for the first and the last node and one in between.
 *
 * Test Sequence:
 *  1. Set expectations on CANOpen mock object. Let the service call getting the brake_test_status object respond
 *     an error for the selected node.
 *  2. Publish fixed joint states.
 *  3. Call brake test service.
 *  4. Verify and clear expectations.
 *
 * Expected Results:
 *  1. -
 *  2. -
 *  3. The brake test service responds with correct error case.
 *  4. There is at least one read request on brake_test_status object for the selected node.
 */
TEST_F(BrakeTestExecutorTest, testBrakeTestStatusServiceResponseFailure)
{
  GetObjectResponse status_resp;
  status_resp.success = false;

  for (size_t node_index : NODE_TEST_SET)
  {
    std::string node{NODE_NAMES_PREFIX + std::to_string(node_index + 1)};

    /**********
     * Step 1 *
     **********/
    // Avoid unmatched expectations
    canopen_chain_node_.expectAnything();

    EXPECT_CALL(canopen_chain_node_, get_obj(AllOf(Field(&GetObjectRequest::node, node),
                                                   Field(&GetObjectRequest::object, BRAKE_TEST_STATUS_OBJECT_INDEX)),
                                             _))
        .Times(AtLeast(1))
        .WillRepeatedly(DoAll(SetArgReferee<1>(status_resp), Return(true)));

    /**********
     * Step 2 *
     **********/
    JointStatesPublisherMock joint_states_pub;
    joint_states_pub.startAsync();

    /**********
     * Step 3 *
     **********/
    BrakeTest srv;
    EXPECT_TRUE(brake_test_srv_client_.call(srv)) << "Failed to call brake test service.";
    EXPECT_EQ(BrakeTestResponse::FAILED_TO_GET_STATUS, srv.response.result);

    /**********
     * Step 4 *
     **********/
    Mock::VerifyAndClearExpectations(&canopen_chain_node_);
  }
}

/**
 * @brief Test execution of brake tests when the brake_test_status is unknown.
 *
 * This is repeated for the first and the last node and one in between.
 *
 * Test Sequence:
 *  1. Set expectations on CANOpen mock object. Let the service call getting the brake_test_status object respond
 *     status unknown for the selected node.
 *  2. Publish fixed joint states.
 *  3. Call brake test service.
 *  4. Verify and clear expectations.
 *
 * Expected Results:
 *  1. -
 *  2. -
 *  3. The brake test service responds with correct error case.
 *  4. There is at least one read request on brake_test_status object for the selected node.
 */
TEST_F(BrakeTestExecutorTest, testBrakeTestStatusUnknown)
{
  GetObjectResponse status_resp;
  status_resp.success = true;
  status_resp.value = "\0";

  for (size_t node_index : NODE_TEST_SET)
  {
    std::string node{NODE_NAMES_PREFIX + std::to_string(node_index + 1)};

    /**********
     * Step 1 *
     **********/
    // Avoid unmatched expectations
    canopen_chain_node_.expectAnything();

    EXPECT_CALL(canopen_chain_node_, get_obj(AllOf(Field(&GetObjectRequest::node, node),
                                                   Field(&GetObjectRequest::object, BRAKE_TEST_STATUS_OBJECT_INDEX)),
                                             _))
        .Times(AtLeast(1))
        .WillRepeatedly(DoAll(SetArgReferee<1>(status_resp), Return(true)));

    /**********
     * Step 2 *
     **********/
    JointStatesPublisherMock joint_states_pub;
    joint_states_pub.startAsync();

    /**********
     * Step 3 *
     **********/
    BrakeTest srv;
    EXPECT_TRUE(brake_test_srv_client_.call(srv)) << "Failed to call brake test service.";
    EXPECT_EQ(BrakeTestResponse::UNKNOWN, srv.response.result);

    /**********
     * Step 4 *
     **********/
    Mock::VerifyAndClearExpectations(&canopen_chain_node_);
  }
}

/**
 * @brief Test execution of brake tests when the brake_test_status is performed.
 *
 * This is repeated for the first and the last node and one in between.
 *
 * Test Sequence:
 *  1. Set expectations on CANOpen mock object. Let the service call getting the brake_test_status object respond
 *     status performed for the selected node.
 *  2. Publish fixed joint states.
 *  3. Call brake test service.
 *  4. Verify and clear expectations.
 *
 * Expected Results:
 *  1. -
 *  2. -
 *  3. The brake test service responds with correct error case.
 *  4. There is at least one read request on brake_test_status object for the selected node.
 */
TEST_F(BrakeTestExecutorTest, testBrakeTestStatusPerformed)
{
  GetObjectResponse status_resp;
  status_resp.success = true;
  status_resp.value = "\x01";

  for (size_t node_index : NODE_TEST_SET)
  {
    std::string node{NODE_NAMES_PREFIX + std::to_string(node_index + 1)};

    /**********
     * Step 1 *
     **********/
    // Avoid unmatched expectations
    canopen_chain_node_.expectAnything();

    EXPECT_CALL(canopen_chain_node_, get_obj(AllOf(Field(&GetObjectRequest::node, node),
                                                   Field(&GetObjectRequest::object, BRAKE_TEST_STATUS_OBJECT_INDEX)),
                                             _))
        .Times(AtLeast(1))
        .WillRepeatedly(DoAll(SetArgReferee<1>(status_resp), Return(true)));

    /**********
     * Step 2 *
     **********/
    JointStatesPublisherMock joint_states_pub;
    joint_states_pub.startAsync();

    /**********
     * Step 3 *
     **********/
    BrakeTest srv;
    EXPECT_TRUE(brake_test_srv_client_.call(srv)) << "Failed to call brake test service.";
    EXPECT_EQ(BrakeTestResponse::PENDING, srv.response.result);

    /**********
     * Step 4 *
     **********/
    Mock::VerifyAndClearExpectations(&canopen_chain_node_);
  }
}

/**
 * @brief Test execution of brake tests when the brake_test_status is not successful.
 *
 * This is repeated for the first and the last node and one in between.
 *
 * Test Sequence:
 *  1. Set expectations on CANOpen mock object. Let the service call getting the brake_test_status object respond
 *     status not successful for the selected node.
 *  2. Publish fixed joint states.
 *  3. Call brake test service.
 *  4. Verify and clear expectations.
 *
 * Expected Results:
 *  1. -
 *  2. -
 *  3. The brake test service responds with correct error case.
 *  4. There is at least one read request on brake_test_status object for the selected node.
 */
TEST_F(BrakeTestExecutorTest, testBrakeTestStatusNotSuccessful)
{
  GetObjectResponse status_resp;
  status_resp.success = true;
  status_resp.value = "\x03";

  for (size_t node_index : NODE_TEST_SET)
  {
    std::string node{NODE_NAMES_PREFIX + std::to_string(node_index + 1)};

    /**********
     * Step 1 *
     **********/
    // Avoid unmatched expectations
    canopen_chain_node_.expectAnything();

    EXPECT_CALL(canopen_chain_node_, get_obj(AllOf(Field(&GetObjectRequest::node, node),
                                                   Field(&GetObjectRequest::object, BRAKE_TEST_STATUS_OBJECT_INDEX)),
                                             _))
        .Times(AtLeast(1))
        .WillRepeatedly(DoAll(SetArgReferee<1>(status_resp), Return(true)));

    /**********
     * Step 2 *
     **********/
    JointStatesPublisherMock joint_states_pub;
    joint_states_pub.startAsync();

    /**********
     * Step 3 *
     **********/
    BrakeTest srv;
    EXPECT_TRUE(brake_test_srv_client_.call(srv)) << "Failed to call brake test service.";
    EXPECT_EQ(BrakeTestResponse::FAILURE, srv.response.result);

    /**********
     * Step 4 *
     **********/
    Mock::VerifyAndClearExpectations(&canopen_chain_node_);
  }
}

/**
 * @brief Test execution of brake tests when the brake_test_status is not actively controlled.
 *
 * This is repeated for the first and the last node and one in between.
 *
 * Test Sequence:
 *  1. Set expectations on CANOpen mock object. Let the service call getting the brake_test_status object respond
 *     status not actively controlled for the selected node.
 *  2. Publish fixed joint states.
 *  3. Call brake test service.
 *  4. Verify and clear expectations.
 *
 * Expected Results:
 *  1. -
 *  2. -
 *  3. The brake test service responds with correct error case.
 *  4. There is at least one read request on brake_test_status object for the selected node.
 */
TEST_F(BrakeTestExecutorTest, testBrakeTestStatusNotActivelyControlled)
{
  GetObjectResponse status_resp;
  status_resp.success = true;
  status_resp.value = "\x04";

  for (size_t node_index : NODE_TEST_SET)
  {
    std::string node{NODE_NAMES_PREFIX + std::to_string(node_index + 1)};

    /**********
     * Step 1 *
     **********/
    // Avoid unmatched expectations
    canopen_chain_node_.expectAnything();

    EXPECT_CALL(canopen_chain_node_, get_obj(AllOf(Field(&GetObjectRequest::node, node),
                                                   Field(&GetObjectRequest::object, BRAKE_TEST_STATUS_OBJECT_INDEX)),
                                             _))
        .Times(AtLeast(1))
        .WillRepeatedly(DoAll(SetArgReferee<1>(status_resp), Return(true)));

    /**********
     * Step 2 *
     **********/
    JointStatesPublisherMock joint_states_pub;
    joint_states_pub.startAsync();

    /**********
     * Step 3 *
     **********/
    BrakeTest srv;
    EXPECT_TRUE(brake_test_srv_client_.call(srv)) << "Failed to call brake test service.";
    EXPECT_EQ(BrakeTestResponse::NO_CONTROL, srv.response.result);

    /**********
     * Step 4 *
     **********/
    Mock::VerifyAndClearExpectations(&canopen_chain_node_);
  }
}

} // namespace brake_test_executor_test

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "unittest_brake_test_executor");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner{2};
  spinner.start();

  testing::InitGoogleMock(&argc, argv);
  return RUN_ALL_TESTS();
}
