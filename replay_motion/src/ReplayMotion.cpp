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


#include "replay_motion/ReplayMotion.hpp"
#include <string>
#include <memory>
#include <vector>
#include <algorithm>


// TODO(Svastits): parameters loop and loop_count
// TODO(Svastits): more csv files with wait between
// TODO(Svastits): csv file: handle if more than 7 columns
// TODO(Svastits): joint limits? -> can get into endless loop

std::string getLastLine(std::ifstream & in)
{
  std::string line;
  while (std::getline(in, line)) {}

  return line;
}

namespace replay_motion
{
ReplayMotion::ReplayMotion(
  const std::string & node_name,
  const rclcpp::NodeOptions & options)
: rclcpp::Node(node_name, options)
{
  csv_in_.open("replay.csv");
  std::string line, value;
  std::vector<double> joint_angles;
  if (std::getline(csv_in_, line)) {
    std::stringstream s(line);
    while (std::getline(s, value, ',')) {
      joint_angles.push_back(std::stod(value));
    }

    reference_ = std::make_shared<sensor_msgs::msg::JointState>();
    reference_->position.resize(7);
    reference_->position = joint_angles;
    measured_joint_state_ = reference_;

    reference_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "reference_joint_state", qos_);

    measured_joint_state_listener_ = this->create_subscription<
      sensor_msgs::msg::JointState>(
      "lbr_joint_state", qos_,
      [this](sensor_msgs::msg::JointState::SharedPtr state) {
        measured_joint_state_ = state;
      });

    int duration_us = static_cast<int>(125000 / rate_);  // default rate is 8Hz (125 ms)
    timer_ = this->create_wall_timer(
      std::chrono::microseconds(duration_us),
      [this]() {
        this->timerCallback();
      });
    RCLCPP_INFO(
      this->get_logger(), "Starting publishing with a rate of %lf Hz",
      static_cast<double>(1000000 / duration_us));
  }

  param_callback_ = this->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & parameters) {
      return this->onParamChange(parameters);
    });

  this->declare_parameter(
    "rate", rclcpp::ParameterValue(rate_));
  this->declare_parameter(
    "repeat_count", rclcpp::ParameterValue(repeat_count_));
}

void ReplayMotion::timerCallback()
{
  if (!reached_start_) {
    double dist_sum = 0;
    sensor_msgs::msg::JointState to_start;
    std::vector<double> joint_error;
    for (int i = 0; i < 7; i++) {
      double dist = reference_->position[i] - measured_joint_state_->position[i];
      dist_sum += pow(dist, 2);
      joint_error.push_back(
        measured_joint_state_->position[i] +
        ((dist > 0) - (dist < 0)) * std::min(0.03 / rate_, abs(dist)));
      // TODO(Svastits): parameter for max speed at beginning
    }
    // (dist > 0) - (dist < 0) is sgn function
    to_start.position = joint_error;
    if (dist_sum < 0.001) {
      reached_start_ = true;
      RCLCPP_INFO(
        this->get_logger(),
        "Robot reached start position, starting the actual motion");
    } else {
      reference_publisher_->publish(to_start);
    }
  } else {
    reference_publisher_->publish(*reference_);

    std::string line, value;
    std::vector<double> joint_angles;
    if (std::getline(csv_in_, line)) {
      std::stringstream s(line);
      while (std::getline(s, value, ',')) {
        joint_angles.push_back(std::stod(value));
      }
      reference_->position = joint_angles;
    } else {
      rclcpp::shutdown();
      RCLCPP_INFO(
        this->get_logger(),
        "End of file reached, stopping publishing");
    }
  }
}

rcl_interfaces::msg::SetParametersResult ReplayMotion::onParamChange(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const rclcpp::Parameter & param : parameters) {
    if (param.get_name() == "rate") {
      result.successful = onRateChangeRequest(param);
    } else if (param.get_name() == "repeat_count") {
      result.successful = onRepeatCountChangeRequest(param);
    } else {
      RCLCPP_ERROR(
        this->get_logger(), "Invalid parameter name %s",
        param.get_name().c_str());
      result.successful = false;
    }
  }
  return result;
}

bool ReplayMotion::onRateChangeRequest(const rclcpp::Parameter & param)
{
  if (param.get_type() !=
    rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE)
  {
    RCLCPP_ERROR(
      this->get_logger(), "Invalid parameter type for parameter %s",
      param.get_name().c_str());
    return false;
  }
  if (param.as_double() < 0.2 || param.as_double() > 5) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Invalid parameter value for parameter %s",
      param.get_name().c_str());
    RCLCPP_ERROR(this->get_logger(), "0.2 < rate < 5 must be true");
    return false;
  }

  rate_ = param.as_double();
  return true;
}

bool ReplayMotion::onRepeatCountChangeRequest(const rclcpp::Parameter & param)
{
  if (param.get_type() !=
    rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER)
  {
    RCLCPP_ERROR(
      this->get_logger(), "Invalid parameter type for parameter %s",
      param.get_name().c_str());
    return false;
  }
  if (reached_start_) {
    RCLCPP_ERROR(
      this->get_logger(),
      "The repeat count can't be changed if motion has already started",
      param.get_name().c_str());
    return false;
  }
  if (param.as_int()) {
    std::ifstream csv_last;
    csv_last.open("replay.csv");
    std::string value;
    std::string line = getLastLine(csv_last);
    std::vector<double> joint_angles;
    std::stringstream s(line);
    while (std::getline(s, value, ',')) {
      joint_angles.push_back(std::stod(value));
    }
    double dist_sum = 0;
    for (int i = 0; i < 7; i++) {
      double dist = reference_->position[i] -
        joint_angles[i];
      dist_sum += pow(dist, 2);

      if (dist_sum > 0.1) {
        RCLCPP_ERROR(
          this->get_logger(),
          "The deviation of start and end of motion is too big");
        RCLCPP_ERROR(
          this->get_logger(),
          "Can't set repeat for this motion");
        return false;
      }
    }
  }
  repeat_count_ = param.as_int();
  return true;
}
}  // namespace replay_motion

int main(int argc, char * argv[])
{
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor executor;
  auto node = std::make_shared<replay_motion::ReplayMotion>(
    "replay_motion",
    rclcpp::NodeOptions());
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
