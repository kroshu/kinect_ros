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

#ifndef KINECT_ROS_TRACE_ENDPOINT__POSEPUBLISHER_HPP_
#define KINECT_ROS_TRACE_ENDPOINT__POSEPUBLISHER_HPP_

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <thread>
#include <vector>

#include "kroshu_ros2_core/ROS2BaseNode.hpp"

namespace pose_debug
{

class PosePublisher : public kroshu_ros2_core::ROS2BaseNode
{
public:
  PosePublisher();
  ~PosePublisher() override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & state) override;

private:
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Pose>::SharedPtr
    pose_publisher_;
  geometry_msgs::msg::Pose position_;
  std::vector<double> orientation_vector;
  rclcpp::Parameter orientation_;
  geometry_msgs::msg::Quaternion orientation_q_ = geometry_msgs::msg::Quaternion();
  rclcpp::QoS qos_ = rclcpp::QoS(rclcpp::KeepLast(1));
  std::thread publish_thread_;
  const std::chrono::milliseconds sleeping_time_ms_ = std::chrono::milliseconds(250);

  void publish_loop();
};

}  // namespace pose_debug

#endif  // KINECT_ROS_TRACE_ENDPOINT__POSEPUBLISHER_HPP_
