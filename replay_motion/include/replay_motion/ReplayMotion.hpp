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
#include "kuka_sunrise_interfaces/srv/set_double.hpp"
#include "kuka_sunrise/internal/service_tools.hpp"


namespace replay_motion
{
class ReplayMotion : public rclcpp::Node
{
public:
  ReplayMotion(const std::string & node_name, const rclcpp::NodeOptions & options);

private:
  bool reached_start_ = false;
  bool valid_ = true;
  int repeat_count_ = 0;  // negative numbers mean repeat infinitely
  int delay_count_ = 0;
  std::vector<double> rates_ = std::vector<double>({1.0});
  std::vector<double> delays_ = std::vector<double>({0.0});
  unsigned int csv_count_ = 1;
  std::vector<std::string> csv_path_;
  std::ifstream csv_in_;
  rclcpp::callback_group::CallbackGroup::SharedPtr cbg_;
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::JointState::SharedPtr reference_;
  sensor_msgs::msg::JointState::SharedPtr measured_joint_state_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr reference_publisher_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr manage_processing_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr measured_joint_state_listener_;
  rclcpp::Client<kuka_sunrise_interfaces::srv::SetDouble>::SharedPtr set_rate_client_;
  rclcpp::QoS qos_ = rclcpp::QoS(rclcpp::KeepLast(1));
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_;

  void timerCallback();
  bool onRatesChangeRequest(const rclcpp::Parameter & param);
  bool onDelaysChangeRequest(const rclcpp::Parameter & param);
  bool processCSV(std::vector<double> & joint_angles, bool last_only = false);
  bool onRepeatCountChangeRequest(const rclcpp::Parameter & param);
  bool setControllerRate(const double & rate);
  bool checkJointLimits(const std::vector<double> & angles);
  rcl_interfaces::msg::SetParametersResult onParamChange(
    const std::vector<rclcpp::Parameter> & parameters);

  std::vector<double> lower_limits_rad_ = std::vector<double>(
    {-2.67, -1.88,
      -2.67, -1.88, -2.67, -1.88, -2.74});
  std::vector<double> upper_limits_rad_ = std::vector<double>(
    {2.67, 1.88,
      2.67, 1.88, 2.67, 1.88, 2.74});
};
}  // namespace replay_motion

#endif  // REPLAY_MOTION__REPLAYMOTION_HPP_
