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

#include <vector>
#include <string>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"


namespace replay_motion
{
class ReplayMotion : public rclcpp::Node
{
public:
  ReplayMotion(const std::string & node_name, const rclcpp::NodeOptions & options);

private:
  bool init_ = true;
  rclcpp::TimerBase::SharedPtr timer_;
  void timerCallback();
  double rate_ = 1;
  std::ifstream csv_in_;
  sensor_msgs::msg::JointState reference_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr reference_publisher_;
  rclcpp::QoS qos_ = rclcpp::QoS(rclcpp::KeepLast(1));
};
}  // namespace replay_motion

#endif  // REPLAY_MOTION__REPLAYMOTION_HPP_
