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

#include "kinect_ros_trace_endpoint/PosePublisher.hpp"
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <thread>
#include <cmath>
#include <chrono>
#include <memory>
#include <vector>

namespace pose_debug
{

PosePublisher::PosePublisher()
: kroshu_ros2_core::ROS2BaseNode("pose_publisher"),
  orientation_vector(4)
{
  qos_.best_effort();
  pose_publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("goal_pos", 1);
  // TODO(Svastits): declare parameters to be able to control with them during runtime
  this->declare_parameter("x_pos", rclcpp::ParameterValue(static_cast<float>(0)));
  this->declare_parameter("y_pos", rclcpp::ParameterValue(static_cast<float>(0)));
  this->declare_parameter("z_pos", rclcpp::ParameterValue(static_cast<float>(1.258)));
  this->declare_parameter("orientation", std::vector<double>{0, 0, 0, 1});
}

PosePublisher::~PosePublisher()
{
  kroshu_ros2_core::ROS2BaseNode::on_shutdown(get_current_state());
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PosePublisher::on_activate(const rclcpp_lifecycle::State & state)
{
  pose_publisher_->on_activate();
  publish_thread_ = std::thread(&PosePublisher::publish_loop, this);
  return kroshu_ros2_core::ROS2BaseNode::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PosePublisher::on_deactivate(const rclcpp_lifecycle::State & state)
{
  pose_publisher_->on_deactivate();
  position_.position.x = 0;
  position_.position.y = 0;
  position_.position.z = 1.258;
  if (publish_thread_.joinable()) {publish_thread_.join();}
  return kroshu_ros2_core::ROS2BaseNode::SUCCESS;
}

void PosePublisher::publish_loop()
{
  while (pose_publisher_->is_activated()) {
    this->get_parameter("x_pos", position_.position.x);   // TODO(Svastits): missing error handling
    this->get_parameter("y_pos", position_.position.y);
    this->get_parameter("z_pos", position_.position.z);
    orientation_vector = {0.0, 0.0, 0.0, 1.0};
    this->get_parameter("orientation", orientation_vector);
    position_.orientation.x = orientation_vector[0];
    position_.orientation.y = orientation_vector[1];
    position_.orientation.z = orientation_vector[2];
    position_.orientation.w = orientation_vector[3];

    float length = sqrt(
      position_.orientation.x * position_.orientation.x + position_.orientation.y *
      position_.orientation.y + position_.orientation.z * position_.orientation.z +
      position_.orientation.w * position_.orientation.w);
    if (length != 0) {
      position_.orientation.x /= length;
      position_.orientation.y /= length;
      position_.orientation.z /= length;
      position_.orientation.w /= length;
    } else {position_.orientation.w = 1.0;}

    pose_publisher_->publish(position_);
    std::this_thread::sleep_for(sleeping_time_ms_);
  }
}

}  // namespace pose_debug

int main(int argc, char const * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  auto node = std::make_shared<pose_debug::PosePublisher>();
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();

  return 0;
}
