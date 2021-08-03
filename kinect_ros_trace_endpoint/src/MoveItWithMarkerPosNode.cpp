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
  arm("iiwa14_arm", moveit_cpp_)
{
  // Get the planning_scene_monitor to publish scene diff's for RViz visualization
  moveit_cpp_->getPlanningSceneMonitor()->providePlanningSceneService();
  moveit_cpp_->getPlanningSceneMonitor()->setPlanningScenePublishingFrequency(25);

  robot_state_publisher_ = this->create_publisher<moveit_msgs::msg::DisplayRobotState>(
    "display_robot_state", 1);
  qos.best_effort();
  msg_strategy =
    std::make_shared<rclcpp::message_memory_strategy::
      MessageMemoryStrategy<visualization_msgs::msg::MarkerArray>>();
  callback = [this](visualization_msgs::msg::MarkerArray::ConstSharedPtr msg)
    {markersReceivedCallback(msg);};
  marker_array_subscriber_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
    "markers",
    qos, callback, rclcpp::SubscriptionOptions(), msg_strategy);
  configureScene();
}

MoveItWithMarkerPosNode::~MoveItWithMarkerPosNode()
{
}

void MoveItWithMarkerPosNode::configureScene()
{
  // Create collision object, planning shouldn't be too easy
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = "URDFLBRiiwa14RobotBase";
  collision_object.id = "box";

  shape_msgs::msg::SolidPrimitive box;
  box.type = box.BOX;
  box.dimensions = {0.1, 0.4, 0.1};

  geometry_msgs::msg::Pose box_pose;
  box_pose.position.x = 0.4;
  box_pose.position.y = 0.0;
  box_pose.position.z = 1.0;

  collision_object.primitives.push_back(box);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  // Add object to planning scene
  {  // Lock PlanningScene
    planning_scene_monitor::LockedPlanningSceneRW scene(moveit_cpp_->getPlanningSceneMonitor());
    scene->processCollisionObjectMsg(collision_object);
  }  // Unlock PlanningScene
}

void MoveItWithMarkerPosNode::markersReceivedCallback(
  visualization_msgs::msg::MarkerArray::ConstSharedPtr msg)
{
  auto handtip_it = std::find_if(
    msg->markers.begin(), msg->markers.end(),
    [](const visualization_msgs::msg::Marker & marker) {
      int joint_id = marker.id % 100;
      return joint_id == static_cast<int>(RightArmJoints::Handtip);
    });

  // Set joint state goal
  RCLCPP_INFO(get_logger(), "Set goal");
  geometry_msgs::msg::PoseStamped goal_pose;
  goal_pose.header.frame_id = "world";
  goal_pose.pose = handtip_it->pose;
  arm.setGoal(goal_pose, "URDFLBRiiwaAxis7");

  // Run actual plan
  RCLCPP_INFO(get_logger(), "Plan to goal");
  const auto plan_solution_1 = arm.plan();
  if (plan_solution_1) {
    RCLCPP_INFO(get_logger(), "arm.execute()");
    arm.execute();
  }
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
