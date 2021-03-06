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

#ifndef PRBT_HARDWARE_SUPPORT_ROS_TEST_HELPER_H
#define PRBT_HARDWARE_SUPPORT_ROS_TEST_HELPER_H

#include <string>
#include <algorithm>

#include <ros/ros.h>
#include <gtest/gtest.h>

namespace prbt_hardware_support
{

/**
 * @brief Blocks until a node defined by node_name comes up.
 * @param node_name
 * @param loop_frequency Frequency at which the system is checked for the node.
 */
inline void waitForNode(std::string node_name, double loop_frequency = 10.0)
{
  ROS_ERROR_STREAM("Waiting for Node " << node_name);
  std::vector<std::string> node_names;
  while (ros::master::getNodes(node_names) &&
         std::find(node_names.begin(), node_names.end(), node_name) == node_names.end())
  {
    ros::Rate(loop_frequency).sleep();
  }
  ROS_ERROR_STREAM("Node " << node_name << " found");
}

/**
 * @brief Blocks until a node defined by node_name can no longer be found.
 * @param node_name
 * @param timeout
 * @param loop_frequency Frequency at which the system is checked for the node.
 */
inline ::testing::AssertionResult waitForNodeShutdown(std::string node_name, double timeout = 1.0, double loop_frequency = 10.0)
{
  std::vector<std::string> node_names;
  auto start_time = ros::Time::now();
  while (ros::master::getNodes(node_names) &&
         std::find(node_names.begin(), node_names.end(), node_name) != node_names.end())
  {
    node_names.clear();
    if (ros::Time::now() > start_time + ros::Duration(timeout))
    {
      return ::testing::AssertionFailure() << "Timed out waiting for shutdown of Node " << node_name;
    }
    ros::Rate(loop_frequency).sleep();
  }
  return ::testing::AssertionSuccess();
}

}  // namespace prbt_hardware_support


#endif // PRBT_HARDWARE_SUPPORT_ROS_TEST_HELPER_H
