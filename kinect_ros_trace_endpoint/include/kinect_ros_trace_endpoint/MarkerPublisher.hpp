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

#ifndef MARKER_MOVEIT__MARKERPUBLISHER_HPP_
#define MARKER_MOVEIT__MARKERPUBLISHER_HPP_

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <thread>

#include "kroshu_ros2_core/ROS2BaseNode.hpp"

namespace marker_moveit
{

class MarkerPublisher : public kroshu_ros2_core::ROS2BaseNode
{
public:
  MarkerPublisher();
  ~MarkerPublisher() override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & state) override;

private:
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    marker_array_publisher_;
  visualization_msgs::msg::MarkerArray marker_array_;
  rclcpp::QoS qos_ = rclcpp::QoS(rclcpp::KeepLast(1));
  std::thread publish_thread_;
  float alpha_rad_ = 0.0;
  visualization_msgs::msg::Marker marker_;
  const float circle_r_ = 0.1;
  const std::chrono::milliseconds sleeping_time_ms_ = std::chrono::milliseconds(250);
  const float circle_step_rad_ = M_PI / 36.0;
  const float z_offset_ = 0.2;

  const int kinect_endpoint_id_ = 16;
  const float x_starter_pos_ = 0.5;

  void publish_loop();
};

}  // namespace marker_moveit

#endif  // MARKER_MOVEIT__MARKERPUBLISHER_HPP_
