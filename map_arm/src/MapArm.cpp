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
  marker_listener_ = this->create_subscription<
    visualization_msgs::msg::MarkerArray>(
    "body_tracking_data", qos_,
    [this](visualization_msgs::msg::MarkerArray::SharedPtr msg) {
      this->markersReceivedCallback(msg);
    });
  reference_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(
    "reference_joint_state", qos_);

  cbg_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  manage_processing_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "system_manager/manage", 1, [this](
      std_msgs::msg::Bool::SharedPtr valid) {manageProcessingCallback(valid);});
  change_state_client_ = this->create_client<std_srvs::srv::Trigger>(
    "system_manager/trigger_change", ::rmw_qos_profile_default, cbg_);

  prev_rel_pos_.x = prev_rel_pos_.y =
    prev_rel_pos_.z = 0;

  param_callback_ = this->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & parameters) {
      return this->onParamChange(parameters);
    });

  const rosbag2_cpp::ConverterOptions converter_options(
    {rmw_get_serialization_format(),
      rmw_get_serialization_format()});
  rosbag_writer_ = std::make_unique<rosbag2_cpp::writers::SequentialWriter>();
  try {
    rosbag_writer_->open(storage_options_, converter_options);
    rosbag_writer_->create_topic(
      {"reference_joint_state", "std_msgs/Float64MultiArray",
        rmw_get_serialization_format(), ""});
    bag_count_++;
  } catch (const std::runtime_error & e) {
    RCLCPP_ERROR(
      this->get_logger(), "Could not open DB for writing");
    RCLCPP_ERROR(
      this->get_logger(), e.what());
    rclcpp::shutdown();
  }

  this->declare_parameter(
    "moving_avg_depth", std::vector<int64_t> {1, 1, 1, 1,
      4, 4, 0});
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

MapArm::~MapArm()
{
  rosbag_writer_->reset();
  std::string old_path = storage_options_.uri + "/" + storage_options_.uri + "_0.db3";
  std::string new_path = storage_options_.uri + "/motion" + std::to_string(bag_count_) +
    ".db3";
  // rename returns 0 if successful
  if (rename(
      old_path.c_str(),
      new_path.c_str()))
  {
    RCLCPP_ERROR(
      get_logger(),
      "Could not rename bagfile");
  }
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

  auto left_hand_it = std::find_if(
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

    if (left_hand_it != msg->markers.end()) {
      auto left_hand = poseDiff(
        left_hand_it->pose.position,
        shoulder_it->pose.position);
      // Start recording if left hand is raised vertically left
      if (left_hand.y > 0.9 && !record_) {
        RCLCPP_INFO(get_logger(), "Starting recording");
        record_ = true;
      }
      // Stop if left hand is raised upwards
      if (left_hand.z > 0.4) {
        RCLCPP_INFO(get_logger(), "Motion stopped with left hand");
        auto future_result = change_state_client_->async_send_request(
          trigger_request_);
        auto future_status = kuka_sunrise::wait_for_result(
          future_result,
          std::chrono::milliseconds(500));
        if (future_status != std::future_status::ready) {
          RCLCPP_ERROR(get_logger(), "Future status not ready, stopping node");
          rclcpp::shutdown();
          return;
        }
        if (!future_result.get()->success) {
          RCLCPP_ERROR(
            get_logger(),
            "Future result not success, stopping node");
          rclcpp::shutdown();
          return;
        }
      }
    } else {
      RCLCPP_WARN(
        get_logger(),
        "Left handtip joint not found, stopping not possible that way");
    }

    if (record_) {
      writeBagFile(reference);
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
      RCLCPP_DEBUG(get_logger(), "Reference published");
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
    auto future_result = change_state_client_->async_send_request(
      trigger_request_);
    auto future_status = kuka_sunrise::wait_for_result(
      future_result,
      std::chrono::milliseconds(500));
    if (future_status != std::future_status::ready) {
      RCLCPP_ERROR(get_logger(), "Future status not ready, stopping node");
      rclcpp::shutdown();
      return;
    }
    if (!future_result.get()->success) {
      RCLCPP_ERROR(get_logger(), "Future result not success, stopping node");
      rclcpp::shutdown();
      return;
    }
  }
}

void MapArm::writeBagFile(const sensor_msgs::msg::JointState & reference)
{
  auto message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
  auto serializer = rclcpp::Serialization<std_msgs::msg::Float64MultiArray>();
  auto serialized_message = rclcpp::SerializedMessage();
  // TODO(Svastits): is this converision necessary? create own type?
  std_msgs::msg::Float64MultiArray ref;
  ref.data = reference.position;
  serializer.serialize_message(&ref, &serialized_message);

  message->serialized_data =
    std::shared_ptr<rcutils_uint8_array_t>(
    new rcutils_uint8_array_t,
    [this](rcutils_uint8_array_t * msg_data) {
      auto fini_return = rcutils_uint8_array_fini(msg_data);
      delete msg_data;
      if (fini_return != RCUTILS_RET_OK) {
        RCLCPP_ERROR_STREAM(
          this->get_logger(),
          "Failed to destroy serialized message " << rcutils_get_error_string().str);
      }
    });
  *message->serialized_data =
    serialized_message.release_rcl_serialized_message();

  message->topic_name = "reference_joint_state";
  message->time_stamp = this->now().nanoseconds();
  rosbag_writer_->write(message);
}

void MapArm::manageProcessingCallback(
  std_msgs::msg::Bool::SharedPtr valid)
{
  if (valid) {
    valid_ = true;
    RCLCPP_INFO(
      this->get_logger(),
      "System manager is active again, continuing motion");
    if (record_) {
      const rosbag2_cpp::ConverterOptions converter_options(
        {rmw_get_serialization_format(),
          rmw_get_serialization_format()});
      try {
        rosbag_writer_->open(storage_options_, converter_options);
        rosbag_writer_->create_topic(
          {"reference_joint_state", "std_msgs/Float64MultiArray",
            rmw_get_serialization_format(), ""});
        bag_count_++;
      } catch (const std::runtime_error & e) {
        RCLCPP_ERROR(
          this->get_logger(), "Could not open DB for writing");
        RCLCPP_ERROR(
          this->get_logger(), e.what());
        rclcpp::shutdown();
      }
    }
  } else {
    valid_ = false;
    RCLCPP_WARN(
      this->get_logger(),
      "LBR state is not 4, reactivate or restart system manager!");
    if (record_) {
      rosbag_writer_->reset();
      std::string old_path = storage_options_.uri + "/" + storage_options_.uri + ".db3";
      std::string new_path = storage_options_.uri + "/motion" + std::to_string(bag_count_) +
        "_0.db3";
      if (rename(
          old_path.c_str(),
          new_path.c_str()))
      {
        RCLCPP_ERROR(
          get_logger(),
          "Could not rename bagfile");
      }
    }
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
