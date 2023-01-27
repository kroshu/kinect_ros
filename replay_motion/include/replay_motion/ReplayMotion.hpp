// Copyright 2022 Aron Svastits
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

#ifndef REPLAY_MOTION__REPLAYMOTION_HPP_
#define REPLAY_MOTION__REPLAYMOTION_HPP_

#include <math.h>
#include <vector>
#include <string>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "rcpputils/filesystem_helper.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "communication_helpers/service_tools.hpp"

#include "kroshu_ros2_core/ROS2BaseNode.hpp"


namespace replay_motion
{
const std::vector<double> lower_limits_rad_ = std::vector<double>(
  {-2.67, -1.88,
    -2.67, -1.88, -2.67, -1.88, -2.74});
const std::vector<double> upper_limits_rad_ = std::vector<double>(
  {2.67, 1.88,
    2.67, 1.88, 2.67, 1.88, 2.74});

class ReplayMotion : public kroshu_ros2_core::ROS2BaseNode
{
public:
  ReplayMotion(const std::string & node_name, const rclcpp::NodeOptions & options);

private:
  int ticks_to_start = 2;
  bool valid_ = true;
  int repeat_count_ = 0;  // negative numbers mean repeat infinitely
  int delay_count_ = 0;
  std::vector<double> rates_ = std::vector<double>({13.0});
  std::vector<double> delays_ = std::vector<double>({0.0});
  unsigned int csv_count_ = 1;
  std::vector<std::string> csv_path_;
  std::ifstream csv_in_;
  rclcpp::CallbackGroup::SharedPtr cbg_;
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::JointState reference_;
  sensor_msgs::msg::JointState::SharedPtr measured_joint_state_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr reference_publisher_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr manage_processing_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr measured_joint_state_listener_;
  rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr set_rate_client_;
  rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr get_rate_client_;
  rclcpp::QoS qos_ = rclcpp::QoS(rclcpp::KeepLast(1));

  rcl_interfaces::srv::SetParameters::Request::SharedPtr set_rate_request_;
  rcl_interfaces::srv::GetParameters::Request::SharedPtr get_rate_request_;
  rcl_interfaces::msg::Parameter controller_rate_;

  void timerCallback();
  bool checkFiles();
  void addParameters();
  void initCommunications();
  bool onRatesChangeRequest(const std::vector<double> & rates);
  bool onDelaysChangeRequest(const std::vector<double> & delays);
  bool processCSV(std::vector<double> & joint_angles, bool last_only = false);
  bool onRepeatCountChangeRequest(const int & repeat_count);
  bool setControllerRate(const double & rate) const;
  bool changeRate(const double & rate, const double & prev_rate);
  bool checkJointLimits(const std::vector<double> & angles) const;

  static constexpr int us_in_sec_ = 1000000;

  // Define start rate for the first tick, because client request gets in deadlock in constructor
  double start_rate_ = 10;
  bool first_flag_ = true;
};
}  // namespace replay_motion

#endif  // REPLAY_MOTION__REPLAYMOTION_HPP_
