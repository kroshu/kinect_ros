// Copyright 2020 Zoltán Rési
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef CONTROL_SYSTEM__SYSTEM_MANAGER_HPP_
#define CONTROL_SYSTEM__SYSTEM_MANAGER_HPP_

#include <vector>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "kuka_sunrise_interfaces/srv/get_state.hpp"

namespace driver_guided_robot
{

class SystemManager : public rclcpp_lifecycle::LifecycleNode
{
public:
  SystemManager(const std::string & node_name, const rclcpp::NodeOptions & options);

  void activateControl();
  void deactivateControl();

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State &);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_error(const rclcpp_lifecycle::State &);

private:
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::JointState>::SharedPtr
    reference_joint_state_publisher_;
  bool changeState(const std::string & node_name, std::uint8_t transition);
  bool changeRobotCommandingState(bool is_active);
  void robotCommandingStateChanged(bool is_active);
  std::vector<rclcpp::Subscription<lifecycle_msgs::msg::Transition>::SharedPtr>
  lifecycle_subscriptions_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr robot_commanding_state_subscription_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr change_robot_commanding_state_client_;
  bool robot_control_active_;
  sensor_msgs::msg::JointState::SharedPtr reference_joint_state_;

  void GetFRIState();
  void MonitoringLoop();
  bool start_ = true;
  bool stop_ = false;
  int lbr_state_ = 0;
  rclcpp::Client<kuka_sunrise_interfaces::srv::GetState>::SharedPtr get_state_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_processing_client_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_change_service_;
  std_srvs::srv::Trigger::Request::SharedPtr trigger_request_ =
    std::make_shared<std_srvs::srv::Trigger::Request>();
  rclcpp::callback_group::CallbackGroup::SharedPtr cbg_;
  rclcpp::QoS qos_;
  std::thread polling_thread_;
  const std::chrono::milliseconds sleeping_time_ms_ = std::chrono::milliseconds(
    200);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn SUCCESS =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ERROR =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn FAILURE =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;

  const std::string JOINT_CONTROLLER = "joint_controller";
  const std::string ROBOT_INTERFACE = "robot_manager";
};


}  // namespace driver_guided_robot

#endif  // CONTROL_SYSTEM__SYSTEM_MANAGER_HPP_