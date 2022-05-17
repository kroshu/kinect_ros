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
#include "replay_motion/Instrumentor.h"
#include <sys/mman.h>
#include <string>
#include <memory>
#include <vector>
#include <algorithm>

std::string getLastLine(std::ifstream & in)
{
  std::string line;
  while (in >> std::ws && std::getline(in, line)) { /* skip lines until last */}

  return line;
}

namespace replay_motion
{
ReplayMotion::ReplayMotion(
  const std::string & node_name,
  const rclcpp::NodeOptions & options)
: kroshu_ros2_core::ROS2BaseNode(node_name, options)
{
  Instrumentor::Instance().beginSession("Replay_motion");
  PROFILE_FUNC();

  set_rate_request_ = std::make_shared<
    rcl_interfaces::srv::SetParameters::Request>();

  get_rate_request_ = std::make_shared<
    rcl_interfaces::srv::GetParameters::Request>();
  get_rate_request_->names.resize(1);
  get_rate_request_->names[0] = "reference_rate";

  auto manage_proc_callback = [this](
    std_msgs::msg::Bool::SharedPtr valid) {
      valid_ = valid->data;
    };
  manage_processing_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "system_manager/manage", 1, manage_proc_callback);

  int i = 0;
  while (rcpputils::fs::exists(
      rcpputils::fs::path(
        "replay/data/motion" + std::to_string(
          i + 1) + ".csv")))
  {
    csv_path_.push_back("replay/data/motion" + std::to_string(i + 1) + ".csv");
    i++;
  }
  if (!csv_path_.size()) {
    RCLCPP_ERROR(this->get_logger(), "File does not exist, stopping node");
    rclcpp::shutdown();
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Found %i '.csv' files to replay", i);
  csv_in_.open(csv_path_[0]);
  if (csv_in_ >> std::ws && csv_in_.peek() == std::ifstream::traits_type::eof()) {
    RCLCPP_ERROR(this->get_logger(), "File is empty, stopping node");
    rclcpp::shutdown();
    return;
  }
  std::vector<double> joint_angles;

  if (!processCSV(joint_angles)) {
    RCLCPP_INFO(this->get_logger(), "Stopping node");
    rclcpp::shutdown();
    return;
  }

  reference_ = std::make_shared<sensor_msgs::msg::JointState>();
  reference_->position.resize(7);
  reference_->position = joint_angles;
  if (!measured_joint_state_) {
    measured_joint_state_ = reference_;
  }
  if (!checkJointLimits(joint_angles)) {
    RCLCPP_ERROR(
      this->get_logger(),
      "First point is exceeding limits, stopping node");
    rclcpp::shutdown();
    return;
  }

  cbg_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  set_rate_client_ = this->create_client<rcl_interfaces::srv::SetParameters>(
    "joint_controller/set_parameters", ::rmw_qos_profile_default, cbg_);

  controller_rate_.name = "reference_rate";
  set_rate_request_->parameters.resize(1);
  set_rate_request_->parameters[0] = controller_rate_;

  get_rate_client_ = this->create_client<rcl_interfaces::srv::GetParameters>(
    "joint_controller/get_parameters", ::rmw_qos_profile_default, cbg_);

  param_callback_ = this->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & parameters) {
	  return getParameterHandler().onParamChange(parameters);
    });

  registerParameter<std::vector<double>>(
    "rates", std::vector<double>(csv_path_.size(), 10.0),
	[this](const std::vector<double> & rates) {
      return this->onRatesChangeRequest(rates);
    });

  // Time to wait before part of motion in seconds
  registerParameter<std::vector<double>>(
    "delays", std::vector<double>(csv_path_.size(), 0.0),
	[this](const std::vector<double> & delays) {
      return this->onDelaysChangeRequest(delays);
    });

  registerParameter<int>(
    "repeat_count", repeat_count_,
	[this](const int & repeats) {
      return this->onRepeatCountChangeRequest(repeats);
    });

  reference_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(
    "reference_joint_state", qos_);

  measured_joint_state_listener_ = this->create_subscription<
    sensor_msgs::msg::JointState>(
    "lbr_joint_state", qos_,
    [this](sensor_msgs::msg::JointState::SharedPtr state) {
      measured_joint_state_ = state;
    });

  // Start timer with default for one tick
  auto duration_us = static_cast<int>(ReplayMotion::us_in_sec_ / start_rate_);
  timer_ = this->create_wall_timer(
    std::chrono::microseconds(duration_us),
    [this]() {
      this->timerCallback();
    });

  if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
    RCLCPP_ERROR(get_logger(), "mlockall error");
    RCLCPP_ERROR(get_logger(), strerror(errno));
  }

  struct sched_param param;
  param.sched_priority = 90;
  if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
    RCLCPP_ERROR(get_logger(), "setscheduler error");
    RCLCPP_ERROR(get_logger(), strerror(errno));
  }
}

ReplayMotion::~ReplayMotion()
{
  Instrumentor::Instance().endSession();
}

void ReplayMotion::timerCallback()
{
  PROFILE_FUNC();
  if (!valid_) {
    RCLCPP_ERROR(
      this->get_logger(), "LBR state is not 4, stopping playback");
    rclcpp::shutdown();
    return;
  }
  // Sync rate parameter with joint_controller
  // valid until start position is reached
  if (first_flag_) {
    first_flag_ = false;
    auto future_result = get_rate_client_->async_send_request(get_rate_request_);
    auto future_status = kuka_sunrise::wait_for_result(
      future_result,
      std::chrono::milliseconds(100));
    if (future_status != std::future_status::ready) {
      RCLCPP_ERROR(
        get_logger(),
        "Future status not ready, could not get rate of joint controller");
      rclcpp::shutdown();
      return;
    }
    double start_rate = future_result.get()->values[0].double_value;
    if (start_rate_ != start_rate) {
      start_rate_ = start_rate;
      timer_->cancel();
      auto duration_us = static_cast<int>(ReplayMotion::us_in_sec_ / start_rate_);
      timer_ = this->create_wall_timer(
        std::chrono::microseconds(duration_us),
        [this]() {
          this->timerCallback();
        });
    }
    RCLCPP_INFO(
      this->get_logger(), "Starting publishing with a rate of %lf Hz",
      start_rate_);
  }
  if (repeat_count_ < 0) {repeat_count_ = -1;}
  if (!reached_start_) {
    double dist_sum = 0;
    sensor_msgs::msg::JointState to_start;
    std::vector<double> joint_error;
    for (int i = 0; i < 7; i++) {
      double dist = reference_->position[i] -
        measured_joint_state_->position[i];
      dist_sum += pow(dist, 2);
      joint_error.push_back(
        measured_joint_state_->position[i] +
        (static_cast<int>(dist > 0) - static_cast<int>(dist < 0)) * std::min(
          0.03, abs(
            dist)));
    }
    // (dist > 0) - (dist < 0) is sgn function
    to_start.position = joint_error;
    if (dist_sum < 0.001) {
      if (!setControllerRate(rates_[0])) {
        rclcpp::shutdown();
        return;
      }
      auto duration_us = static_cast<int>(ReplayMotion::us_in_sec_ / rates_[0]);
      if (rates_[0] != start_rate_) {
        timer_->cancel();
        timer_ = this->create_wall_timer(
          std::chrono::microseconds(duration_us),
          [this]() {
            this->timerCallback();
          });
      }
      delay_count_ = static_cast<int>(delays_[0] * ReplayMotion::us_in_sec_ / duration_us);

      reached_start_ = true;
      RCLCPP_INFO(
        this->get_logger(),
        "Robot reached start position, starting the actual motion");
    } else {
      reference_publisher_->publish(to_start);
    }
  } else {
    reference_->header.stamp = this->now();
    reference_publisher_->publish(*reference_);

    std::vector<double> joint_angles;
    if (!processCSV(joint_angles)) {
      RCLCPP_INFO(this->get_logger(), "Stopping node");
      rclcpp::shutdown();
      return;
    }
    reference_->position = joint_angles;
  }
}

bool ReplayMotion::processCSV(
  std::vector<double> & joint_angles,
  bool last_only)
{
  PROFILE_FUNC();
  // Getting the next line of a csv based on the parameters
  if (delay_count_ && !last_only) {
    delay_count_--;
    joint_angles = reference_->position;
    return true;
  }
  bool file_change = false;
  std::string line;
  std::string value;
  if (last_only) {
    std::ifstream csv_last;
    csv_last.open(csv_path_.back());
    line = getLastLine(csv_last);
  } else if (!std::getline(csv_in_, line)) {
    if (csv_count_ != csv_path_.size()) {
      RCLCPP_INFO(this->get_logger(), "End of file reached, switching to next one");
      file_change = true;
      auto duration_us = static_cast<int>(ReplayMotion::us_in_sec_ / rates_[csv_count_]);
      if (rates_[csv_count_] != rates_[csv_count_ - 1]) {
        timer_->cancel();
        timer_ = this->create_wall_timer(
          std::chrono::microseconds(duration_us),
          [this]() {
            this->timerCallback();
          });
        if (!setControllerRate(rates_[csv_count_])) {
          return false;
        }
      }
      delay_count_ = static_cast<int>(delays_[csv_count_] * ReplayMotion::us_in_sec_ / duration_us);
      csv_in_.close();
      csv_in_.open(csv_path_[csv_count_]);
      if (csv_in_ >> std::ws && !std::getline(csv_in_, line)) {
        RCLCPP_ERROR(
          this->get_logger(),
          "Next file is empty");
        return false;
      }
      csv_count_++;
    } else if (repeat_count_) {
      RCLCPP_INFO(this->get_logger(), "End of motion reached, repeating");
      RCLCPP_INFO(this->get_logger(), "Repeats remaining: %i", repeat_count_);
      auto duration_us = static_cast<int>(ReplayMotion::us_in_sec_ / rates_[0]);
      if (rates_[csv_count_ - 1] != rates_[0]) {
        timer_->cancel();
        timer_ = this->create_wall_timer(
          std::chrono::microseconds(duration_us),
          [this]() {
            this->timerCallback();
          });
        if (!setControllerRate(rates_[0])) {
          return false;
        }
      }
      delay_count_ = static_cast<int>(delays_[0] * ReplayMotion::us_in_sec_ / duration_us);
      repeat_count_--;
      csv_in_.close();
      csv_in_.open(csv_path_[0]);
      std::getline(csv_in_, line);
      csv_count_ = 1;
    } else {
      RCLCPP_INFO(
        this->get_logger(),
        "End of file reached");
      return false;
    }
  }

  // Processing the line and converting to double array
  std::stringstream s(line);
  double double_value;
  while (std::getline(s, value, ',')) {
    try {
      double_value = std::stod(value);
    } catch (const std::invalid_argument & ia) {
      RCLCPP_ERROR(this->get_logger(), ia.what());
      RCLCPP_ERROR(
        this->get_logger(),
        "Could not convert to double");
      return false;
    }
    if (!isnan(double_value)) {
      joint_angles.push_back(double_value);
    }
  }
  if (joint_angles.size() != 7) {
    RCLCPP_ERROR(
      this->get_logger(),
      "The number of joint values is not 7");
    return false;
  }
  if (file_change) {
    double dist_max = 0;
    for (int i = 0; i < 7; i++) {
      double dist = reference_->position[i] -
        joint_angles[i];
      if (dist_max < abs(dist)) {dist_max = abs(dist);}
    }

    if (dist_max > 0.1) {
      RCLCPP_ERROR(
        this->get_logger(),
        "The distance to start of next motion is too big");
      return false;
    }
  }
  return true;
}

bool ReplayMotion::onRatesChangeRequest(const std::vector<double> & rates)
{
  PROFILE_FUNC();

  if (rates.size() != csv_path_.size()) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Invalid parameter array length for parameter rates");
    return false;
  }

  for (auto & rate : rates) {
    if (rate < 0.2 || rate > 5) {
      RCLCPP_ERROR(this->get_logger(), "0.2 < rate < 5 must be true");
      return false;
    }
  }
  rates_ = rates;
  return true;
}

bool ReplayMotion::setControllerRate(const double & rate) const
{
  PROFILE_FUNC();
  set_rate_request_->parameters[0].value = rclcpp::ParameterValue(rate).to_value_msg();
  auto future_result = set_rate_client_->async_send_request(set_rate_request_);
  auto future_status = kuka_sunrise::wait_for_result(
    future_result,
    std::chrono::milliseconds(100));
  if (future_status != std::future_status::ready) {
    RCLCPP_ERROR(
      get_logger(),
      "Future status not ready, could not set rate of joint controller");
    return false;
  }
  if (!future_result.get()->results[0].successful) {
    RCLCPP_ERROR(
      get_logger(),
      "Future result not success, could not set rate of joint controller");
    return false;
  }
  return true;
}


bool ReplayMotion::onDelaysChangeRequest(const std::vector<double> & delays)
{
  PROFILE_FUNC();

  if (delays.size() != csv_path_.size()) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Invalid parameter array length for parameter delays");
    return false;
  }

  for (auto delay : delays) {
    if (delay < 0) {
      RCLCPP_ERROR(this->get_logger(), "Delay must be positive");
      return false;
    }
  }
  delays_ = delays;
  return true;
}

bool ReplayMotion::onRepeatCountChangeRequest(const int & repeat_count)
{
  PROFILE_FUNC();
  if (repeat_count) {
    std::vector<double> joint_angles;
    if (!processCSV(joint_angles, true)) {
      return false;
    }
    double dist_max = 0;
    for (int i = 0; i < 7; i++) {
      double dist = reference_->position[i] -
        joint_angles[i];
      if (dist_max < abs(dist)) {dist_max = abs(dist);}
    }

    if (dist_max > 0.1) {
      RCLCPP_ERROR(
        this->get_logger(),
        "The deviation of start and end of motion is too big");
      RCLCPP_ERROR(this->get_logger(), "Can't set repeat for this motion");
      return false;
    }
  }
  repeat_count_ = repeat_count;
  return true;
}

bool ReplayMotion::checkJointLimits(const std::vector<double> & angles) const
{
  PROFILE_FUNC();
  for (int i = 0; i < 7; i++) {
    if (angles[i] < lower_limits_rad_[i] || angles[i] > upper_limits_rad_[i]) {
      return false;
    }
  }
  return true;
}
}  // namespace replay_motion

int main(int argc, char * argv[])
{
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<replay_motion::ReplayMotion>(
    "replay_motion",
    rclcpp::NodeOptions());
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
