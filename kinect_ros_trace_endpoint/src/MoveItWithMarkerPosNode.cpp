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

#include "kinect_ros_trace_endpoint/MoveItWithMarkerPosNode.hpp"

#include <rclcpp/rclcpp.hpp>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <string>
#include <memory>

namespace marker_moveit
{

MoveItWithMarkerPosNode::MoveItWithMarkerPosNode(
  const std::string & name,
  const rclcpp::NodeOptions & node_options)
: rclcpp::Node(name, "", node_options),
  moveit_cpp_(std::make_shared<moveit::planning_interface::MoveItCpp>(
      static_cast<rclcpp::Node::SharedPtr>(this))),
  arm_("iiwa14_arm", moveit_cpp_)
{
  qos_.best_effort();
  msg_strategy =
    std::make_shared<rclcpp::message_memory_strategy::
      MessageMemoryStrategy<geometry_msgs::msg::Pose>>();
  callback_ = [this](geometry_msgs::msg::Pose::ConstSharedPtr msg)
    {goalReceivedCallback(msg);};
  goal_pos_subscriber_ = this->create_subscription<geometry_msgs::msg::Pose>(
    "goal_pos",
    qos_, callback_, rclcpp::SubscriptionOptions(), msg_strategy);
}

MoveItWithMarkerPosNode::~MoveItWithMarkerPosNode()
{
}


void MoveItWithMarkerPosNode::goalReceivedCallback(
  geometry_msgs::msg::Pose::ConstSharedPtr msg)
{
  // Set joint state goal
  RCLCPP_INFO(get_logger(), "Set goal");
  geometry_msgs::msg::PoseStamped goal_pose;
  goal_pose.header.frame_id = "world";
  goal_pose.pose.position = msg->position;
  goal_pose.pose.orientation = msg->orientation;
  RCLCPP_INFO(get_logger(), std::to_string(goal_pose.pose.orientation.x));
  arm_.setGoal(goal_pose, "URDFLBRiiwaAxis7");

  // Run actual plan
  RCLCPP_INFO(get_logger(), "Plan to goal");
  const auto plan_solution_1 = arm_.plan();
  if (plan_solution_1) {
    RCLCPP_INFO(get_logger(), "arm.execute()");
    arm_.execute();
  } else {RCLCPP_WARN(get_logger(), "Pose cannot be reached");}
}

}  // namespace marker_moveit

int main(int argc, char const * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  rclcpp::NodeOptions node_options;
  // This enables loading undeclared parameters
  // best practice would be to declare parameters in the corresponding classes
  // and provide descriptions about expected use
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<marker_moveit::MoveItWithMarkerPosNode>(
    "moveit_with_markerpos",
    node_options);
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();

  return 0;
}
