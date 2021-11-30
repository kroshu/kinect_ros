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
: rclcpp::Node(node_name, options)
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
      if (request->data) {
        valid_ = true;
        RCLCPP_INFO(
          this->get_logger(),
          "System manager is active again, continuing motion");
      } else {
        valid_ = false;
        RCLCPP_WARN(
          this->get_logger(),
          "LBR state is not 4, reactivate or restart system manager!");
      }
      response->success = true;
    };
  manage_processing_service_ = this->create_service<std_srvs::srv::SetBool>(
    "manage_processing", manage_proc_callback);
  change_state_client_ = this->create_client<std_srvs::srv::Trigger>(
    "system_manager/trigger_change");

  prev_rel_pos_.x = prev_rel_pos_.y =
    prev_rel_pos_.z = 0;

  this->declare_parameter(
    "moving_avg_depth", std::vector<int64_t> {1, 1, 1, 1,
      4, 4, 0});

  param_callback_ = this->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & parameters) {
      return this->onParamChange(parameters);
    });
}

rcl_interfaces::msg::SetParametersResult MapArm::onParamChange(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const rclcpp::Parameter & param : parameters) {
    if (param.get_name() == "moving_avg_depth") {
      result.successful = onMovingAvgChangeRequest(param);
    } else {
      RCLCPP_ERROR(
        this->get_logger(), "Invalid parameter name %s",
        param.get_name().c_str());
      result.successful = false;
    }
  }
  return result;
}

bool MapArm::onMovingAvgChangeRequest(const rclcpp::Parameter & param)
{
  if (motion_started_) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Parameter %s cannot be changed when motion has started",
      param.get_name().c_str());
  }
  if (param.get_type() !=
    rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER_ARRAY)
  {
    RCLCPP_ERROR(
      this->get_logger(), "Invalid parameter type for parameter %s",
      param.get_name().c_str());
    return false;
  }
  if (param.as_integer_array().size() != 7) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Invalid parameter array length for parameter %s",
      param.get_name().c_str());
    return false;
  }
  for (int64_t ma : param.as_integer_array()) {
    if (ma < 0) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Invalid parameter value for parameter %s", param.get_name().c_str());
      RCLCPP_ERROR(
        this->get_logger(),
        "Moving average values must be greater than 0");
      return false;
    }
  }
  moving_avg_depth_ = param.as_integer_array();
  return true;
}

void MapArm::markersReceivedCallback(
  visualization_msgs::msg::MarkerArray::SharedPtr msg)
{
  motion_started_ = true;
  if (!valid_) {
    return;
  }
  // Invalidate, if more than 1 person can be seen
  if (msg->markers.size() > 32) {
    RCLCPP_WARN(get_logger(), "More bodies in view, invalidating commands");
    return;
  }

  // Getting the required joint positions
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

  if (handtip_it != msg->markers.end() &&
    wrist_it != msg->markers.end() &&
    shoulder_it != msg->markers.end() &&
    elbow_it != msg->markers.end() &&
    hand_it != msg->markers.end())
  {
    sensor_msgs::msg::JointState reference;
    std::vector<double> joint_state(7);

    auto elbow_rel_pos = poseDiff(
      elbow_it->pose.position,
      shoulder_it->pose.position);

    calculateJoints12(joint_state, elbow_rel_pos);

    auto wrist_rel_pos = poseDiff(
      wrist_it->pose.position,
      elbow_it->pose.position);

    calculateJoints34(joint_state, wrist_rel_pos);

    // Calculate joints 5 and 6
    auto handtip_rel_pos = poseDiff(
      handtip_it->pose.position,
      wrist_it->pose.position);

    calculateJoints56(joint_state, handtip_rel_pos);

    // Joint 7 is constant 0, as it would need thumb position, which is inaccurate
    joint_state[6] = 0;
    reference.position = joint_state;

    // Stop if left hand is raised
    if (stop_it != msg->markers.end()) {
      auto left_stop = poseDiff(stop_it->pose.position, shoulder_it->pose.position);
      if (left_stop.z > 0.4) {
        RCLCPP_INFO(get_logger(), "Motion stopped with left hand");
        change_state_client_->async_send_request(trigger_request_);
      }
    } else {
      RCLCPP_WARN(
        get_logger(),
        "Left handtip joint not found, stopping not possible that way");
    }

    // If cartesian distance is small, do not send new commands
    auto rel_pos = poseDiff(
      handtip_it->pose.position,
      shoulder_it->pose.position);
    auto delta = poseDiff(rel_pos, prev_rel_pos_);
    double delta_len = sqrt(
      delta.x * delta.x + delta.y * delta.y + delta.z * delta.z);

    if (delta_len > 0.01) {
      prev_rel_pos_ = rel_pos;
      RCLCPP_INFO(get_logger(), "Reference published");
      reference_publisher_->publish(reference);
    } else {
      RCLCPP_INFO(
        get_logger(), "Skipping frame, distance is only %f [cm]",
        delta_len * 100);
    }
    prev_joint_state_ = joint_state;
  } else {
    RCLCPP_WARN(
      get_logger(),
      "Missing joint from hand, stopping motion");
    change_state_client_->async_send_request(trigger_request_);
  }
}

void MapArm::calculateJoints12(
  std::vector<double> & joint_state,
  const geometry_msgs::msg::Point & elbow_rel_pos)
{
  if (abs(elbow_rel_pos.x) > 0.03 || abs(elbow_rel_pos.y) > 0.03) {
    joint_state[0] = atan2(elbow_rel_pos.y, elbow_rel_pos.x);
  } else {
    joint_state[0] = prev_joint_state_[0];
  }
  joint_state[1] = atan2(
    sqrt(pow(elbow_rel_pos.x, 2) + pow(elbow_rel_pos.y, 2)), elbow_rel_pos.z);

  // Moving average
  for (int i = 0; i < 2; i++) {
    int factor = static_cast<int>(moving_avg_depth_[i]) + 1;
    joint_state[i] = joint_state[i] / factor +
      prev_joint_state_[i] * (factor - 1) / factor;
  }
}

void MapArm::calculateJoints34(
  std::vector<double> & joint_state,
  const geometry_msgs::msg::Point & wrist_rel_pos)
{
  Eigen::AngleAxisd rot1(-joint_state[0], Eigen::Vector3d::UnitZ());
  Eigen::AngleAxisd rot2(-joint_state[1], Eigen::Vector3d::UnitY());
  Eigen::Matrix3d rot = rot2.toRotationMatrix() * rot1.toRotationMatrix();
  Eigen::Vector3d wrist_rel_pos_glob(wrist_rel_pos.x, wrist_rel_pos.y, wrist_rel_pos.z);
  Eigen::Vector3d w_rel_pos_loc = rot * wrist_rel_pos_glob;

  if (abs(w_rel_pos_loc[0]) > 0.03 || abs(w_rel_pos_loc[1]) > 0.03) {
    joint_state[2] = atan2(-w_rel_pos_loc[1], -w_rel_pos_loc[0]);
  } else {
    joint_state[2] = prev_joint_state_[2];
  }
  joint_state[3] = atan2(
    sqrt(pow(w_rel_pos_loc[0], 2) + pow(w_rel_pos_loc[1], 2)),
    w_rel_pos_loc[2]);

  for (int i = 2; i < 4; i++) {
    int factor = static_cast<int>(moving_avg_depth_[i]) + 1;
    joint_state[i] = joint_state[i] / factor +
      prev_joint_state_[i] * (factor - 1) / factor;
  }
}

void MapArm::calculateJoints56(
  std::vector<double> & joint_state,
  const geometry_msgs::msg::Point & handtip_rel_pos)
{
  Eigen::AngleAxisd rot1(-joint_state[0], Eigen::Vector3d::UnitZ());
  Eigen::AngleAxisd rot2(-joint_state[1], Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd rot3(-joint_state[2], Eigen::Vector3d::UnitZ());
  Eigen::AngleAxisd rot4(joint_state[3], Eigen::Vector3d::UnitY());
  Eigen::Matrix3d rot = rot4.toRotationMatrix() * rot3.toRotationMatrix() *
    rot2.toRotationMatrix() * rot1.toRotationMatrix();
  Eigen::Vector3d h_rel_pos_glob(handtip_rel_pos.x, handtip_rel_pos.y,
    handtip_rel_pos.z);
  Eigen::Vector3d h_rel_pos_loc = rot * h_rel_pos_glob;

  joint_state[4] = atan2(abs(h_rel_pos_loc[1]), abs(h_rel_pos_loc[0]));

  if (abs(h_rel_pos_loc[1]) > abs(h_rel_pos_loc[0])) {
    if (h_rel_pos_loc[1] > 0) {
      joint_state[5] = atan2(
        sqrt(pow(h_rel_pos_loc[0], 2) + pow(h_rel_pos_loc[1], 2)),
        h_rel_pos_loc[2]);
    } else {
      joint_state[5] = -atan2(
        sqrt(pow(h_rel_pos_loc[0], 2) + pow(h_rel_pos_loc[1], 2)),
        h_rel_pos_loc[2]);
    }
  } else {
    if (h_rel_pos_loc[0] > 0) {
      joint_state[5] = atan2(
        sqrt(pow(h_rel_pos_loc[0], 2) + pow(h_rel_pos_loc[1], 2)),
        h_rel_pos_loc[2]);
    } else {
      joint_state[5] = -atan2(
        sqrt(pow(h_rel_pos_loc[0], 2) + pow(h_rel_pos_loc[1], 2)),
        h_rel_pos_loc[2]);
    }
  }

  for (int i = 4; i < 6; i++) {
    int factor = static_cast<int>(moving_avg_depth_[i]) + 1;
    joint_state[i] = joint_state[i] / factor +
      prev_joint_state_[i] * (factor - 1) / factor;
  }
}
}  // namespace filter_points

int main(int argc, char * argv[])
{
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);
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
