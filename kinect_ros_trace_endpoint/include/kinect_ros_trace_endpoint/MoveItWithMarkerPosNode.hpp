// Copyright 2020 Gergely Kovács
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

#ifndef MARKER_MOVEIT__MOVEITWITHMARKERPOSNODE_HPP_
#define MARKER_MOVEIT__MOVEITWITHMARKERPOSNODE_HPP_

#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <moveit/robot_state/conversions.h>

#include <rclcpp/rclcpp.hpp>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <thread>
#include <string>

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
  moveit::planning_interface::MoveItCppPtr moveit_cpp_;
  moveit::planning_interface::PlanningComponent arm;
  rclcpp::Publisher<moveit_msgs::msg::DisplayRobotState>::SharedPtr robot_state_publisher_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_subscriber_;
  rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(1));
  rclcpp::message_memory_strategy::MessageMemoryStrategy<visualization_msgs::msg::MarkerArray>::
  SharedPtr msg_strategy;
  std::function<void(visualization_msgs::msg::MarkerArray::ConstSharedPtr msg)> callback;

  enum class RightArmJoints {Clavicle = 11, Shoulder, Elbow, Wrist, Hand, Handtip};

  void configureScene();
  void markersReceivedCallback(
    visualization_msgs::msg::MarkerArray::ConstSharedPtr msg);
};

}  // namespace marker_moveit

#endif  // MARKER_MOVEIT__MOVEITWITHMARKERPOSNODE_HPP_
