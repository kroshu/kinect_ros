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

#include "kinect_ros_trace_endpoint/MarkerPublisher.hpp"
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <thread>
#include <cmath>
#include <chrono>
#include <memory>

namespace marker_moveit
{

MarkerPublisher::MarkerPublisher()
: kroshu_ros2_core::ROS2BaseNode("marker_publisher")
{
  qos_.best_effort();
  marker_array_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "body_tracking_data", 1);

  marker_array_.markers.clear();
  marker_.header.frame_id = "world";
  marker_.id = kinect_endpoint_id_;
  marker_.pose.position.x = x_starter_pos_;
  marker_array_.markers.emplace_back(marker_);
}

MarkerPublisher::~MarkerPublisher()
{
  kroshu_ros2_core::ROS2BaseNode::on_shutdown(get_current_state());
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MarkerPublisher::on_activate(const rclcpp_lifecycle::State & state)
{
  marker_array_publisher_->on_activate();
  publish_thread_ = std::thread(&MarkerPublisher::publish_loop, this);
  return kroshu_ros2_core::ROS2BaseNode::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MarkerPublisher::on_deactivate(const rclcpp_lifecycle::State & state)
{
  marker_array_publisher_->on_deactivate();
  if (publish_thread_.joinable()) {publish_thread_.join();}
  return kroshu_ros2_core::ROS2BaseNode::SUCCESS;
}

void MarkerPublisher::publish_loop()
{
  while (marker_array_publisher_->is_activated()) {
    marker_array_.markers[0].pose.position.y = circle_r_ * std::cos(alpha_rad_);
    marker_array_.markers[0].pose.position.z = z_offset_ + circle_r_ * std::sin(alpha_rad_);

    marker_array_publisher_->publish(marker_array_);
    alpha_rad_ += circle_step_rad_;
    std::this_thread::sleep_for(sleeping_time_ms_);
  }
}

}  // namespace marker_moveit

int main(int argc, char const * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  auto node = std::make_shared<marker_moveit::MarkerPublisher>();
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();

  return 0;
}
