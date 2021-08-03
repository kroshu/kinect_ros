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
  cbg_ = this->create_callback_group(
    rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
  get_state_client_ = this->create_client<
    kuka_sunrise_interfaces::srv::GetState>(
    "robot_control/get_fri_state");
  this->declare_parameter(
    "fake_execution",
    rclcpp::ParameterValue(static_cast<bool>(false)));
  prev_rel_pose_.position.x = prev_rel_pose_.position.y =
    prev_rel_pose_.position.z = 0;
}

void FilterPoints::CoordinateTransform(geometry_msgs::msg::Point & pos1)
{
  float tmp = pos1.x;
  pos1.x = -pos1.z;
  pos1.z = -pos1.y;
  pos1.y = tmp;
}

geometry_msgs::msg::Quaternion FilterPoints::ToQuaternion(
  const geometry_msgs::msg::Point & or1,
  const geometry_msgs::msg::Point & or2,
  const geometry_msgs::msg::Point & or3)
{
  geometry_msgs::msg::Quaternion result;
  if (or1.x + or2.y + or3.z > 0) {
    result.w = sqrt(1 + or1.x + or2.y + or3.z) / 2;
    result.x = (or2.z - or3.y) / (4 * result.w);
    result.y = (or3.x - or1.z) / (4 * result.w);
    result.z = (or1.y - or2.x) / (4 * result.w);
  } else if (or1.x > or2.y && or1.x > or3.z) {
    float s = 2 * sqrt(1 + or1.x - or2.y - or3.z);
    result.w = (or2.z - or3.y) / s;
    result.x = s / 4;
    result.y = (or1.y + or2.x) / s;
    result.z = (or3.x + or1.z) / s;
  } else if (or2.y > or3.z) {
    float s = 2 * sqrt(1 - or1.x + or2.y - or3.z);
    result.w = (or3.x - or1.z) / s;
    result.x = (or1.y + or2.x) / s;
    result.y = s / 4;
    result.z = (or2.z + or3.y) / s;
  } else {
    float s = 2 * sqrt(1 - or1.x - or2.y + or3.z);
    result.w = (or1.y - or2.x) / s;
    result.x = (or3.x + or1.z) / s;
    result.y = (or2.z + or3.y) / s;
    result.z = s / 4;
  }
  float length = sqrt(
    result.x * result.x + result.y * result.y + result.z * result.z +
    result.w * result.w);
  result.x /= length;
  result.y /= length;
  result.z /= length;
  result.w /= length;

  return result;
}

geometry_msgs::msg::Point FilterPoints::PoseDiff(
  const geometry_msgs::msg::Point & pos1,
  const geometry_msgs::msg::Point & pos2, bool coord_trans)
{
  geometry_msgs::msg::Point result;
  result.x = pos1.x - pos2.x;
  result.y = pos1.y - pos2.y;
  result.z = pos1.z - pos2.z;

  if (coord_trans) {
    CoordinateTransform(result);
  }

  return result;
}

geometry_msgs::msg::Point FilterPoints::CrossProduct(
  const geometry_msgs::msg::Point & pos1,
  const geometry_msgs::msg::Point & pos2, bool normalize)
{
  geometry_msgs::msg::Point result;
  result.x = pos1.y * pos2.z - pos1.z * pos2.y;
  result.y = -pos1.x * pos2.z + pos1.z * pos2.x;
  result.z = pos1.x * pos2.y - pos1.y * pos2.x;

  if (normalize) {
    float length = sqrt(
      result.x * result.x + result.y * result.y +
      result.z * result.z);
    result.x /= length;
    result.y /= length;
    result.z /= length;
  }

  return result;
}

void FilterPoints::ChangeState(
  const std::string & node_name,
  std::uint8_t transition)
{
  if (!fake_execution_) {
    auto client = this->create_client<lifecycle_msgs::srv::ChangeState>(
      node_name + "/change_state", qos_.get_rmw_qos_profile(), cbg_);
    auto request = std::make_shared<
      lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = transition;
    if (!client->wait_for_service(std::chrono::milliseconds(2000))) {
      RCLCPP_ERROR(get_logger(), "Wait for service failed");
    }
    auto future_result = client->async_send_request(request);
    auto future_status = kuka_sunrise::wait_for_result(
      future_result,
      std::chrono::milliseconds(3000));
    if (future_status != std::future_status::ready) {
      RCLCPP_ERROR(get_logger(), "Future status not ready");
    }
    if (!future_result.get()->success) {
      RCLCPP_ERROR(get_logger(), "Future result not success");
    }
  } else {
    RCLCPP_WARN(get_logger(), "Shutting down node in fake execution mode");
    rclcpp::shutdown();
  }
}

void FilterPoints::GetFRIState()
{
  while (!get_state_client_->wait_for_service(std::chrono::milliseconds(1000))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(
      this->get_logger(),
      "service not available, waiting again...");
  }
  auto request = std::make_shared<
    kuka_sunrise_interfaces::srv::GetState::Request>();

  auto response_received_callback =
    [this](
    rclcpp::Client<kuka_sunrise_interfaces::srv::GetState>::SharedFuture future) {
      auto result = future.get();
      lbr_state_ = result->data;
      RCLCPP_INFO(this->get_logger(), "State: %i", lbr_state_);
    };
  auto future_result = get_state_client_->async_send_request(
    request,
    response_received_callback);
}

void FilterPoints::markersReceivedCallback(
  visualization_msgs::msg::MarkerArray::SharedPtr msg)
{
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
    ChangeState(
      "system_manager",
      lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
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
      orientation_y_ = PoseDiff(thumb_pose_.position, wrist_pose_.position);
      orientation_x_ = CrossProduct(orientation_y_, orientation_z_);
      orientation_y_ = CrossProduct(orientation_z_, orientation_x_);
      rel_pose_.orientation = ToQuaternion(
        orientation_x_, orientation_y_,
        orientation_z_);

      if (stop_it != msg->markers.end()) {
        stop_pose_ = stop_it->pose;
        left_stop_ = PoseDiff(stop_pose_.position, pelvis_pose_.position);
        if (left_stop_.z > 0.6) {
          RCLCPP_INFO(get_logger(), "Motion stopped with left hand");
          ChangeState(
            "system_manager",
            lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
        }
      } else {
        RCLCPP_WARN(
          get_logger(),
          "Left handtip joint not found, stopping not possible that way");
      }


      // TODO(Svastits): missing error handling
      this->get_parameter("fake_execution", fake_execution_);
      if (!fake_execution_) {
        GetFRIState();
      }
      if (4 == lbr_state_ || fake_execution_) {
        stop_ = false;
        auto delta = PoseDiff(
          rel_pose_.position,
          prev_rel_pose_.position);
        float delta_len = sqrt(
          delta.x * delta.x + delta.y * delta.y +
          delta.z * delta.z);
        if (delta_len > 0.1) {
          goal_pos_publisher_->publish(rel_pose_);
          prev_rel_pose_ = rel_pose_;
          RCLCPP_INFO(get_logger(), "x: %f", rel_pose_.position.x);
          RCLCPP_INFO(get_logger(), "y: %f", rel_pose_.position.y);
          RCLCPP_INFO(get_logger(), "z: %f", rel_pose_.position.z);
        } else {
          RCLCPP_INFO(
            get_logger(),
            "Skipping frame, distance is only %f [cm]",
            delta_len);
        }

      } else {
        RCLCPP_ERROR(get_logger(), "FRI State is: %i", lbr_state_);

        // Stop only if the state is not 4 for two consecutive cycles
        // to avoid unnecessary shutdowns
        if (!start_ && stop_) {
          rclcpp::shutdown();
        }
        stop_ = true;
      }
    } else {
      RCLCPP_WARN(
        get_logger(),
        "Pelvis joint not found, skipping frame");
      ChangeState(
        "system_manager",
        lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
    }
  } else {
    RCLCPP_WARN(
      get_logger(),
      "Right handtip joint not found, skipping frame");
    ChangeState(
      "system_manager",
      lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
  }
  if (start_) {
    start_ = false;
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
