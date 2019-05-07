/*
 * Copyright (c) 2018 Pilz GmbH & Co. KG
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

#include <atomic>
#include <algorithm>
#include <mutex>
#include <thread>

#include <gtest/gtest.h>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <prbt_hardware_support/brake_test_utils.h>
#include <prbt_hardware_support/brake_test_utils_exception.h>

namespace brake_test_utils_test
{

using namespace prbt_hardware_support;
using sensor_msgs::JointState;
using sensor_msgs::JointStateConstPtr;
using sensor_msgs::JointStatePtr;

static const std::string JOINT_STATES_TOPIC_NAME{"/joint_states"};
static constexpr unsigned int JOINT_STATES_TOPIC_QUEUE_SIZE{1};

/**
 * @brief Checks for identical names and positions in joint state messages.
 */
static ::testing::AssertionResult compareJointStateMessages(const JointStateConstPtr &msg1, const JointStateConstPtr &msg2)
{
  if (msg1->name.size() != msg2->name.size())
  {
    return ::testing::AssertionFailure() << "Joint numbers in joint state messages do not match.";
  }
  if (!std::equal(msg1->name.begin(), msg1->name.end(), msg2->name.begin(),
                  [](std::string name1, std::string name2) { return name1 == name2; }))
  {
    return ::testing::AssertionFailure() << "Joint names in joint state messages do not match.";
  }
  if (msg1->position.size() != msg2->position.size())
  {
    return ::testing::AssertionFailure() << "Joint numbers in joint state messages do not match.";
  }
  if (!BrakeTestUtils::compareJointStatePositions(msg1, msg2))
  {
    return ::testing::AssertionFailure() << "Joint positions in joint state messages do not match.";
  }

  return ::testing::AssertionSuccess();
}

/**
 * @brief Asynchronously publishes a message on the /joint_states topic with rate ~100Hz.
 */
class JointStatesPublisherMock
{
public:
  JointStatesPublisherMock();

  ~JointStatesPublisherMock();

  /**
   * @brief Start a new thread publishing joint states.
   *
   * @param move If true, a movement is simulated, otherwise the positions do not change.
   */
  void startAsync(bool move = false);

  void terminate();

  /**
   * @brief Obtain the message that is published next.
   */
  JointStateConstPtr getNextMessage();

private:
  void start(bool positions_fixed);

private:
  ros::NodeHandle nh_;
  ros::Publisher joint_states_pub_;
  std::thread thread_;
  std::atomic_bool terminate_;
  std::mutex msg_mutex_;
  JointState msg_;
};

JointStatesPublisherMock::JointStatesPublisherMock()
{
  joint_states_pub_ = nh_.advertise<JointState>(JOINT_STATES_TOPIC_NAME, JOINT_STATES_TOPIC_QUEUE_SIZE);
  msg_.name = {"joint1", "joint2"};
  msg_.position = {0.1, -0.11};
}

JointStatesPublisherMock::~JointStatesPublisherMock()
{
  terminate();
}

void JointStatesPublisherMock::startAsync(bool move)
{
  terminate_ = false;
  thread_ = std::thread{ [this, move]{ this->start(move); } };
}

void JointStatesPublisherMock::terminate()
{
  terminate_ = true;
  if (thread_.joinable())
  {
    thread_.join();
  }
}

JointStateConstPtr JointStatesPublisherMock::getNextMessage()
{
  std::lock_guard<std::mutex> lock(msg_mutex_);
  JointStateConstPtr msg(new JointState(msg_));
  return msg;
}

void JointStatesPublisherMock::start(bool move)
{
  while (!terminate_)
  {
    {
      std::lock_guard<std::mutex> lock(msg_mutex_);
      JointStateConstPtr msg(new JointState(msg_));
      joint_states_pub_.publish(msg);
      if (move)
      {
        // change positions; reaches limit after > 100 seconds
        msg_.position.at(0) = std::min(1000.0, msg_.position.at(0)+0.1);
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

/**
 * Test the compareJointStatePositions utility function
 *
 * Test Sequence:
 *  1. Compare two equal joint states
 *  2. Compare two joint states with a difference in all joints slightly below the tolerance
 *  3. Compare two joint_states with a difference in one joint slightly above the tolerance,
 *     repeat the test for all joints.
 *
 * Expected Results:
 *  1. Returns true
 *  2. Returns true
 *  3. Always returns false
 */
TEST(BrakeTestUtilsTest, testCompareJointStatePositions)
{
  /**********
   * Step 1 *
   **********/
  JointStatePtr msg1{boost::make_shared<JointState>()};
  msg1->name = {"joint1", "joint2"};
  msg1->position = {0.1, 0.11};

  JointStatePtr msg2{boost::make_shared<JointState>()};
  msg2->name = {"joint1", "joint2"};
  msg2->position = {0.1, 0.11};

  EXPECT_TRUE(BrakeTestUtils::compareJointStatePositions(msg1, msg2));

  /**********
   * Step 2 *
   **********/
  double tolerance = 0.0001;
  for (size_t i = 0; i < msg2->position.size(); ++i)
  {
    msg2->position[i] += 0.9 * tolerance;
  }

  EXPECT_TRUE(BrakeTestUtils::compareJointStatePositions(msg1, msg2, tolerance));

  /**********
   * Step 3 *
   **********/
  for (size_t i = 0; i < msg2->position.size(); ++i)
  {
    for (size_t j = 0; j < msg2->position.size(); ++j)
    {
      msg2->position[j] = msg1->position[j];
    }
    msg2->position[i] += 1.1 * tolerance;

    EXPECT_FALSE(BrakeTestUtils::compareJointStatePositions(msg1, msg2, tolerance));
  }
}

/**
 * Test the getCurrentJointStates utility function
 *
 * Test Sequence:
 *  1. Call getCurrentJointStates() without publishing to the joint_states topic
 *  2. Call getCurrentJointStates() and publish on the joint_states topic asynchronously
 *
 * Expected Results:
 *  1. Exception is thrown
 *  2. Published message is obtained
 */
TEST(BrakeTestUtilsTest, testGetCurrentJointStates)
{
  /**********
   * Step 1 *
   **********/
  EXPECT_THROW(BrakeTestUtils::getCurrentJointStates(), GetCurrentJointStatesException);

  /**********
   * Step 2 *
   **********/
  JointStatesPublisherMock joint_states_pub;
  auto expected_msg = joint_states_pub.getNextMessage();
  joint_states_pub.startAsync();

  try
  {
    auto msg = BrakeTestUtils::getCurrentJointStates();
    EXPECT_TRUE(compareJointStateMessages(msg, expected_msg));
  }
  catch (const std::exception &e)
  {
    ADD_FAILURE() << e.what();
  }

  joint_states_pub.terminate();
}

/**
 * Test the detectRobotMotion utility function
 *
 * Test Sequence:
 *  1. Publish fixed joint states and call detectRobotMotion()
 *  2. Publish changing joint states and call detectRobotMotion()
 *
 * Expected Results:
 *  1. Returns false.
 *  2. Returns true.
 */
TEST(BrakeTestUtilsTest, testDetectRobotMotion)
{
  /**********
   * Step 1 *
   **********/
  JointStatesPublisherMock joint_states_pub;
  joint_states_pub.startAsync();

  EXPECT_FALSE(BrakeTestUtils::detectRobotMotion());

  joint_states_pub.terminate();

  /**********
   * Step 2 *
   **********/
  joint_states_pub.startAsync(true);

  EXPECT_TRUE(BrakeTestUtils::detectRobotMotion());
}

} // namespace brake_test_utils_test

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "unittest_brake_test_utils");
  ros::NodeHandle nh;

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}