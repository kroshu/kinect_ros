// Copyright 2021 Aron Svastits
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//   http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>
#include <string>
#include "filter_points/FilterPoints.hpp"

namespace filter_points
{

FilterPoints::FilterPoints(
  const std::string & node_name,
  const rclcpp::NodeOptions & options)
: rclcpp::Node(node_name, options), qos_(rclcpp::KeepLast(1))
{
  auto callback = [this](
    visualization_msgs::msg::MarkerArray::SharedPtr msg) {
      this->markersReceivedCallback(msg);
    };
  marker_listener_ = this->create_subscription<
    visualization_msgs::msg::MarkerArray>(
    "body_tracking_data", qos_,
    callback);
  goal_pos_publisher_ = this->create_publisher<geometry_msgs::msg::Pose>(
    "goal_pos", qos_);

  auto manage_proc_callback = [this](
    const std::shared_ptr<rmw_request_id_t> request_header,
    std_srvs::srv::SetBool::Request::SharedPtr request,
    std_srvs::srv::SetBool::Response::SharedPtr response) {
      (void) request_header;
      response->success = true;
      if (request->data) {
        valid_ = true;
      } else {
        valid_ = false;
      }
    };
  manage_processing_service_ = this->create_service<std_srvs::srv::SetBool>(
    "manage_processing", manage_proc_callback);
  change_state_client_ = this->create_client<std_srvs::srv::Trigger>(
    "system_manager/trigger_change");
  prev_rel_pose_.position.x = prev_rel_pose_.position.y =
    prev_rel_pose_.position.z = 0;
  valid_ = true;
}

void FilterPoints::markersReceivedCallback(
  visualization_msgs::msg::MarkerArray::SharedPtr msg)
{
  if (!valid_) {
    return;
  }
  auto handtip_it =
    std::find_if(
    msg->markers.begin(), msg->markers.end(),
    [](const visualization_msgs::msg::Marker & marker) {
      int joint_id = marker.id % 100;
      return joint_id ==
      static_cast<int>(BODY_TRACKING_JOINTS::HANDTIP_RIGHT);
    });

  auto pelvis_it = std::find_if(
    msg->markers.begin(), msg->markers.end(),
    [](const visualization_msgs::msg::Marker & marker) {
      int joint_id = marker.id % 100;
      return joint_id ==
      static_cast<int>(BODY_TRACKING_JOINTS::PELVIS);
    });

  auto wrist_it = std::find_if(
    msg->markers.begin(), msg->markers.end(),
    [](const visualization_msgs::msg::Marker & marker) {
      int joint_id = marker.id % 100;
      return joint_id ==
      static_cast<int>(BODY_TRACKING_JOINTS::WRIST_RIGHT);
    });

  auto thumb_it = std::find_if(
    msg->markers.begin(), msg->markers.end(),
    [](const visualization_msgs::msg::Marker & marker) {
      int joint_id = marker.id % 100;
      return joint_id ==
      static_cast<int>(BODY_TRACKING_JOINTS::THUMB_RIGHT);
    });

  auto second_it = std::find_if(
    msg->markers.begin(), msg->markers.end(),
    [](const visualization_msgs::msg::Marker & marker) {
      int joint_id = marker.id / 100;
      return joint_id != 1;
    });

  auto stop_it = std::find_if(
    msg->markers.begin(), msg->markers.end(),
    [](const visualization_msgs::msg::Marker & marker) {
      int joint_id = marker.id % 100;
      return joint_id ==
      static_cast<int>(BODY_TRACKING_JOINTS::HANDTIP_LEFT);
    });

  if (second_it != msg->markers.end()) {
    RCLCPP_WARN(get_logger(), "Two bodies in view, deactivating control");
    change_state_client_->async_send_request(trigger_request_);
  }

  if (handtip_it != msg->markers.end() && thumb_it != msg->markers.end() &&
    wrist_it != msg->markers.end())
  {
    if (pelvis_it != msg->markers.end()) {
      pelvis_pose_ = pelvis_it->pose;
      handtip_pose_ = handtip_it->pose;
      wrist_pose_ = wrist_it->pose;
      thumb_pose_ = thumb_it->pose;

      rel_pose_.position = PoseDiff(
        handtip_pose_.position,
        pelvis_pose_.position);
      orientation_z_ = PoseDiff(
        handtip_pose_.position,
        wrist_pose_.position);
      orientation_y_ = PoseDiff(
        thumb_pose_.position,
        wrist_pose_.position);
      orientation_x_ = CrossProduct(orientation_y_, orientation_z_);
      orientation_y_ = CrossProduct(orientation_z_, orientation_x_);
      rel_pose_.orientation = ToQuaternion(
        orientation_x_, orientation_y_,
        orientation_z_);

      if (stop_it != msg->markers.end()) {
        stop_pose_ = stop_it->pose;
        left_stop_ = PoseDiff(
          stop_pose_.position,
          pelvis_pose_.position);
        if (left_stop_.z > 0.6) {
          RCLCPP_INFO(get_logger(), "Motion stopped with left hand");
          change_state_client_->async_send_request(trigger_request_);
        }
      } else {
        RCLCPP_WARN(
          get_logger(),
          "Left handtip joint not found, stopping not possible that way");
      }
      auto delta = PoseDiff(rel_pose_.position, prev_rel_pose_.position);
      float delta_len = sqrt(
        delta.x * delta.x + delta.y * delta.y + delta.z * delta.z);
      if (delta_len > 0.05) {
        goal_pos_publisher_->publish(rel_pose_);
        prev_rel_pose_ = rel_pose_;
        RCLCPP_DEBUG(get_logger(), "x: %f", rel_pose_.position.x);
        RCLCPP_DEBUG(get_logger(), "y: %f", rel_pose_.position.y);
        RCLCPP_DEBUG(get_logger(), "z: %f", rel_pose_.position.z);
      } else {
        RCLCPP_INFO(
          get_logger(),
          "Skipping frame, distance is only %f [cm]",
          delta_len * 100);
      }
    } else {
      RCLCPP_WARN(get_logger(), "Pelvis joint not found, skipping frame");
      change_state_client_->async_send_request(trigger_request_);
    }
  } else {
    RCLCPP_WARN(
      get_logger(),
      "Right handtip joint not found, skipping frame");
    change_state_client_->async_send_request(trigger_request_);
  }
}
}  // namespace filter_points

int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<filter_points::FilterPoints>(
    "filter_points",
    rclcpp::NodeOptions());
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
