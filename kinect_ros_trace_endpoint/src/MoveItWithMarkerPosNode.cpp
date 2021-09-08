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

#include "kinect_ros_trace_endpoint/MoveItWithMarkerPosNode.hpp"

#include <tf2_eigen/tf2_eigen.h>

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <memory>
#include <vector>
#include <chrono>


namespace marker_moveit
{

MoveItWithMarkerPosNode::MoveItWithMarkerPosNode(
  const std::string & name,
  const rclcpp::NodeOptions & node_options)
: rclcpp::Node(name, "", node_options), moveit_cpp_(
    std::make_shared<moveit_cpp::MoveItCpp>(
      static_cast<rclcpp::Node::SharedPtr>(this))), arm_(
    "iiwa14_arm", moveit_cpp_)
{
  qos_.best_effort();
  msg_strategy = std::make_shared<
    rclcpp::message_memory_strategy::MessageMemoryStrategy<
      geometry_msgs::msg::Pose>>();
  callback_ = [this](geometry_msgs::msg::Pose::ConstSharedPtr msg) {
      goalReceivedCallback(msg);
    };
  goal_pos_subscriber_ = this->create_subscription<geometry_msgs::msg::Pose>(
    "goal_pos", qos_, callback_, rclcpp::SubscriptionOptions(),
    msg_strategy);
  reference_joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
    "reference_joint_state", 1);
  joint_names_ = {"URDFLBRiiwa14Joint1", "URDFLBRiiwa14Joint2",
    "URDFLBRiiwaJoint3", "URDFLBRiiwaJoint4", "URDFLBRiiwaJoint5",
    "URDFLBRiiwaJoint6", "URDFLBRiiwaJoint7"};
  prev_ref_.header.stamp.sec = 0;
  best_ref_.position = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::vector<std::string> tip {"URDFLBRiiwaAxis7"};
  inverse_kinematics_ = kdl_kinematics_plugin::KDLKinematicsPlugin();
  if (!inverse_kinematics_.initialize(
      this->shared_from_this(),
      *moveit_cpp_->getRobotModel(), "iiwa14_arm",
      "URDFLBRiiwa14RobotBase", tip, 0.1))
  {
    RCLCPP_ERROR(get_logger(), "Failed to initialize");
  }
}

MoveItWithMarkerPosNode::~MoveItWithMarkerPosNode()
{
}

void MoveItWithMarkerPosNode::goalReceivedCallback(
  geometry_msgs::msg::Pose::ConstSharedPtr msg)
{
  moveit_msgs::msg::MoveItErrorCodes err_code;
  geometry_msgs::msg::Pose ik_pose;
  ik_pose.position = msg->position;
  ik_pose.orientation = msg->orientation;
  std::vector<double> seed_values;
  std::vector<double> solution;

  // TODO(Svastits): maybe include this in the while loop?(may increase time considerably)
  // is it the right position?
  auto state = moveit_cpp_->getCurrentState(0.1);
  for (auto & it : joint_names_) {
    seed_values.push_back(*state->getJointPositions(it));
  }

  best_dist_ = INFINITY;
  auto start_time = std::chrono::steady_clock::now();
  std::vector<double> prev_pos = prev_ref_.position;
  int i = 0;
  while (std::chrono::steady_clock::now() - start_time <
    std::chrono::milliseconds(50))
  {
    i++;
    if (i % 2 && prev_ref_.header.stamp.sec != 0) {
      inverse_kinematics_.searchPositionIK(
        ik_pose, prev_pos, 0.1,
        solution, err_code);
    } else {
      inverse_kinematics_.searchPositionIK(ik_pose, seed_values, 0.1, solution, err_code);
    }

    if (err_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Pose cannot be reached");
      break;
    }

    if (prev_ref_.header.stamp.sec == 0) {
      best_ref_.position = solution;
      break;
    }
    Eigen::Map<Eigen::VectorXd> current_plan(&solution[0], 7);
    Eigen::Map<Eigen::VectorXd> prev_plan(&prev_pos[0], 7);
    Eigen::Map<Eigen::VectorXd> actual(&seed_values[0], 7);
    double current_dist = (current_plan - prev_plan).squaredNorm();
    double actual_dist = (current_plan - actual).squaredNorm();
    current_dist = 0.8 * current_dist + 0.2 * actual_dist;
    // TODO(Svastits): adjust weights for planned vs actual
    if (current_dist < best_dist_) {
      best_dist_ = current_dist;
      best_ref_.position = solution;
    }
  }
  RCLCPP_ERROR_STREAM(
    get_logger(), "Best distance: " << best_dist_ << " after " << i << " iterations");
  sensor_msgs::msg::JointState reference;
  reference.position = best_ref_.position;
  reference.header.stamp = this->now();
  reference.header.frame_id = "world";
  reference.name = joint_names_;
  prev_ref_ = reference;
  reference_joint_pub_->publish(reference);
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
    "moveit_with_markerpos", node_options);
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();

  return 0;
}
