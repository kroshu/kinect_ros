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
#include <memory>
#include <string>
#include <cmath>
#include <vector>


// TODO(Svastits): subscribe to joint state and go to starting point slowly !!
// TODO(Svastits): parameter rate
// TODO(Svastits): parameters loop and loop_count
// TODO(Svastits): more csv files with wait between

namespace replay_motion
{

ReplayMotion::ReplayMotion(const std::string & node_name, const rclcpp::NodeOptions & options)
: rclcpp::Node(node_name, options)
{
  reference_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(
    "reference_joint_state", qos_);

  csv_in_.open("replay.csv");
  auto callback = [this]() {
      this->timerCallback();
    };

  int duration_us = static_cast<int>(125000 / rate_);  // default rate is 8Hz (125 ms)
  timer_ = this->create_wall_timer(std::chrono::microseconds(duration_us), callback);
  RCLCPP_INFO(
    this->get_logger(), "Starting publishing with a rate of %lf Hz",
    static_cast<double>(1000000 / duration_us));
}

void ReplayMotion::timerCallback()
{
  if (!init_) {reference_publisher_->publish(reference_);} else {init_ = false;}
  std::string line, value;
  std::vector<double> joint_angles;
  if (std::getline(csv_in_, line)) {
    std::stringstream s(line);
    while (std::getline(s, value, ',')) {
      joint_angles.push_back(std::stod(value));
    }
    reference_.position = joint_angles;
  } else {
    rclcpp::shutdown();
    RCLCPP_INFO(this->get_logger(), "End of file reached, stopping publishing");
  }
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
