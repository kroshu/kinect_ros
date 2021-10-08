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


#include "map_arm/MapArm.hpp"
#include <memory>
#include <string>
#include <cmath>
#include <vector>


namespace filter_points
{

MapArm::MapArm(const std::string & node_name, const rclcpp::NodeOptions & options)
: rclcpp::Node(node_name, options), valid_(true), prev_joint_state_(7), qos_(rclcpp::KeepLast(1))
{
  auto callback = [this](
    visualization_msgs::msg::MarkerArray::SharedPtr msg) {
      this->markersReceivedCallback(msg);
    };
  marker_listener_ = this->create_subscription<
    visualization_msgs::msg::MarkerArray>(
    "body_tracking_data", qos_,
    callback);
  reference_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(
    "reference_joint_state", qos_);

  auto manage_proc_callback = [this](
    const std::shared_ptr<rmw_request_id_t> request_header,
    std_srvs::srv::SetBool::Request::SharedPtr request,
    std_srvs::srv::SetBool::Response::SharedPtr response) {
      (void) request_header;
      if (request->data) {valid_ = true;} else {valid_ = false;}
      response->success = true;
    };
  manage_processing_service_ = this->create_service<std_srvs::srv::SetBool>(
    "manage_processing", manage_proc_callback);
  change_state_client_ = this->create_client<std_srvs::srv::Trigger>(
    "system_manager/trigger_change");
}

void MapArm::markersReceivedCallback(
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

  auto shoulder_it =
    std::find_if(
    msg->markers.begin(), msg->markers.end(),
    [](const visualization_msgs::msg::Marker & marker) {
      int joint_id = marker.id % 100;
      return joint_id ==
      static_cast<int>(BODY_TRACKING_JOINTS::SHOULDER_RIGHT);
    });

  auto elbow_it = std::find_if(
    msg->markers.begin(), msg->markers.end(),
    [](const visualization_msgs::msg::Marker & marker) {
      int joint_id = marker.id % 100;
      return joint_id ==
      static_cast<int>(BODY_TRACKING_JOINTS::ELBOW_RIGHT);
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

  auto hand_it = std::find_if(
    msg->markers.begin(), msg->markers.end(),
    [](const visualization_msgs::msg::Marker & marker) {
      int joint_id = marker.id % 100;
      return joint_id ==
      static_cast<int>(BODY_TRACKING_JOINTS::HAND_RIGHT);
    });

  auto stop_it = std::find_if(
    msg->markers.begin(), msg->markers.end(),
    [](const visualization_msgs::msg::Marker & marker) {
      int joint_id = marker.id % 100;
      return joint_id ==
      static_cast<int>(BODY_TRACKING_JOINTS::HANDTIP_LEFT);
    });

  if (msg->markers.size() > 32) {
    RCLCPP_WARN(get_logger(), "More bodies in view, invalidating commands");
    // change_state_client_->async_send_request(trigger_request_);
    valid_ = false;
  } else {valid_ = true;}

  if (handtip_it != msg->markers.end() && thumb_it != msg->markers.end() &&
    wrist_it != msg->markers.end() &&
    shoulder_it != msg->markers.end() &&
    elbow_it != msg->markers.end() &&
    hand_it != msg->markers.end())
  {
    sensor_msgs::msg::JointState reference;
    std::vector<double> joint_state(7);
    auto elbow_rel_pos = PoseDiff(
      elbow_it->pose.position,
      shoulder_it->pose.position);
    if (abs(elbow_rel_pos.x) > 0.03 || abs(elbow_rel_pos.y) > 0.03) {
      joint_state[0] = atan2(elbow_rel_pos.y, elbow_rel_pos.x);
    } else {
      joint_state[0] = prev_joint_state_[0];
    }
    joint_state[1] = atan2(
      sqrt(pow(elbow_rel_pos.x, 2) + pow(elbow_rel_pos.y, 2)),
      elbow_rel_pos.z);

    auto wrist_rel_pos = PoseDiff(
      wrist_it->pose.position,
      elbow_it->pose.position);

    Eigen::AngleAxisf rot1(-joint_state[0], Eigen::Vector3f::UnitZ());
    Eigen::AngleAxisf rot2(-joint_state[1], Eigen::Vector3f::UnitY());
    Eigen::Matrix3f rot = rot2.toRotationMatrix() * rot1.toRotationMatrix();
    Eigen::Vector3f global_pos(wrist_rel_pos.x, wrist_rel_pos.y, wrist_rel_pos.z);
    Eigen::Vector3f e_rel_pos = rot * global_pos;

    if (abs(e_rel_pos[0]) > 0.03 || abs(e_rel_pos[0]) > 0.03) {
      joint_state[2] = atan2(-e_rel_pos[1], -e_rel_pos[0]);
    } else {
      joint_state[2] = prev_joint_state_[2];
    }
    joint_state[3] = atan2(
      sqrt(pow(e_rel_pos[0], 2) + pow(e_rel_pos[1], 2)),
      e_rel_pos[2]);


    auto handtip_rel_pos = PoseDiff(
      handtip_it->pose.position,
      wrist_it->pose.position);

    Eigen::AngleAxisf rot2_1(-joint_state[2], Eigen::Vector3f::UnitZ());
    Eigen::AngleAxisf rot2_2(joint_state[3], Eigen::Vector3f::UnitY());
    Eigen::Matrix3f rot_2 = rot2_2.toRotationMatrix() * rot2_1.toRotationMatrix() * rot;
    Eigen::Vector3f h_global_pos(handtip_rel_pos.x, handtip_rel_pos.y,
      handtip_rel_pos.z);
    Eigen::Vector3f h_rel_pos = rot_2 * h_global_pos;

    RCLCPP_INFO_STREAM(
      get_logger(),
      h_rel_pos[0] << "  " << h_rel_pos[1] << "  " << h_rel_pos[2]);

    RCLCPP_INFO_STREAM(
      get_logger(),
      sqrt(pow(h_rel_pos[0], 2) + pow(h_rel_pos[1], 2) + pow(h_rel_pos[2], 2)));

    joint_state[4] = atan2(abs(h_rel_pos[1]), abs(h_rel_pos[0]));

    if (abs(h_rel_pos[1]) > abs(h_rel_pos[0])) {
      if (h_rel_pos[1] > 0) {
        joint_state[5] = atan2(
          sqrt(pow(h_rel_pos[0], 2) + pow(h_rel_pos[1], 2)),
          h_rel_pos[2]);
      } else {
        joint_state[5] = -atan2(
          sqrt(pow(h_rel_pos[0], 2) + pow(h_rel_pos[1], 2)),
          h_rel_pos[2]);
      }
    } else {
      if (h_rel_pos[0] > 0) {
        joint_state[5] = atan2(
          sqrt(pow(h_rel_pos[0], 2) + pow(h_rel_pos[1], 2)),
          h_rel_pos[2]);
      } else {
        joint_state[5] = -atan2(
          sqrt(pow(h_rel_pos[0], 2) + pow(h_rel_pos[1], 2)),
          h_rel_pos[2]);
      }
    }

    joint_state[6] = 0;
    if (stop_it != msg->markers.end()) {
      stop_pose_ = stop_it->pose;
      left_stop_ = PoseDiff(stop_pose_.position, shoulder_pose_.position);
      if (left_stop_.z > 0.6) {
        RCLCPP_INFO(get_logger(), "Motion stopped with left hand");
        change_state_client_->async_send_request(trigger_request_);
      }
    } else {
      RCLCPP_WARN(
        get_logger(),
        "Left handtip joint not found, stopping not possible that way");
    }

    reference.position = joint_state;
    if (valid_) {reference_publisher_->publish(reference);}
    prev_joint_state_ = joint_state;
  } else {
    RCLCPP_WARN(
      get_logger(),
      "Missing joint from hand, stopping motion");
    change_state_client_->async_send_request(trigger_request_);
  }
}
}  // namespace filter_points

int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<filter_points::MapArm>(
    "map_arm",
    rclcpp::NodeOptions());
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
