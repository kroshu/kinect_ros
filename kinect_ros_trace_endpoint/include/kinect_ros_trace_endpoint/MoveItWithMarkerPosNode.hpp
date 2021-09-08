// Copyright 2021 Áron Svastits
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef KINECT_ROS_TRACE_ENDPOINT__MOVEITWITHMARKERPOSNODE_HPP_
#define KINECT_ROS_TRACE_ENDPOINT__MOVEITWITHMARKERPOSNODE_HPP_

#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/kdl_kinematics_plugin/kdl_kinematics_plugin.h>

#include <rclcpp/rclcpp.hpp>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <thread>
#include <string>
#include <vector>

namespace marker_moveit
{

class MoveItWithMarkerPosNode : public rclcpp::Node
{
public:
  explicit MoveItWithMarkerPosNode(
    const std::string & name,
    const rclcpp::NodeOptions & node_options);
  ~MoveItWithMarkerPosNode() override;

private:
  moveit_cpp::MoveItCppPtr moveit_cpp_;
  moveit_cpp::PlanningComponent arm_;
  sensor_msgs::msg::JointState prev_ref_;
  sensor_msgs::msg::JointState best_ref_;
  kdl_kinematics_plugin::KDLKinematicsPlugin inverse_kinematics_;
  double best_dist_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr goal_pos_subscriber_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr reference_joint_pub_;
  rclcpp::QoS qos_ = rclcpp::QoS(rclcpp::KeepLast(1));
  rclcpp::message_memory_strategy::MessageMemoryStrategy<geometry_msgs::msg::Pose>::
  SharedPtr msg_strategy;
  std::vector<std::string> joint_names_;
  std::function<void(geometry_msgs::msg::Pose::ConstSharedPtr msg)> callback_;


  void goalReceivedCallback(
    geometry_msgs::msg::Pose::ConstSharedPtr msg);
};

}  // namespace marker_moveit

#endif  // KINECT_ROS_TRACE_ENDPOINT__MOVEITWITHMARKERPOSNODE_HPP_
