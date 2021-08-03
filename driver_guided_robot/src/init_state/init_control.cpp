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


#include "init_state/init_control.hpp"
#include <math.h>

#include <vector>
#include <string>
#include <memory>

namespace driver_guided_robot
{

double d2r(double degrees)
{
  return degrees / 180 * M_PI;
}

InitControl::InitControl(const std::string & node_name, const rclcpp::NodeOptions & options)
: LifecycleNode(node_name, options),
  lower_limits_rad_(7),
  upper_limits_rad_(7),
  changing_joint_(false),
  turning_velocity_increment_({1.0 * M_PI / 180,
      0.6 * M_PI / 180,
      3.0 * M_PI / 180,
      0.6 * M_PI / 180,
      3.0 * M_PI / 180,
      3.0 * M_PI / 180,
      3.0 * M_PI / 180}),
  elapsed_time_treshold_(100000000),
  last_time_(RCL_ROS_TIME)
{
  auto qos = rclcpp::QoS(rclcpp::KeepAll());
  reference_joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(
    "reference_joint_state", qos);
  // Get actual state of joints instead of default zeros, or else fails when it tries to bring it to
  // zero pos quickly

  reference_joint_state_ = std::make_shared<sensor_msgs::msg::JointState>();
//  reference_joint_state_->velocity.assign(7, 0.003);
  reference_joint_state_->position.resize(7);
  this->declare_parameter(
    "lower_limits_deg", rclcpp::ParameterValue(
      std::vector<double>(
        {-170, -120, -170, -120, -170,
          -120, -175})));
  this->declare_parameter(
    "upper_limits_deg", rclcpp::ParameterValue(
      std::vector<double>(
        {170, 120, 170, 120, 170, 120,
          175})));

  param_callback = this->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & parameters)
    {return this->onParamChange(parameters);});
  parameter_set_access_rights_.emplace(
    "lower_limits_deg", ParameterSetAccessRights {true, true,
      true, false});
  parameter_set_access_rights_.emplace(
    "upper_limits_deg", ParameterSetAccessRights {true, true,
      true, false});
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn InitControl::
on_configure(
  const rclcpp_lifecycle::State & state)
{
  (void)state;
  onLowerLimitsChangeRequest(this->get_parameter("lower_limits_deg"));
  onUpperLimitsChangeRequest(this->get_parameter("upper_limits_deg"));
  return SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn InitControl::
on_cleanup(
  const rclcpp_lifecycle::State & state)
{
  (void)state;
  reference_joint_state_->position.assign(7, 0);
  active_joint_ = 0;
  return SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn InitControl::
on_activate(
  const rclcpp_lifecycle::State & state)
{
  (void)state;
  reference_joint_state_publisher_->on_activate();
  last_time_ = this->now();
  reference_joint_state_publisher_->publish(*reference_joint_state_);
  return SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn InitControl::
on_deactivate(
  const rclcpp_lifecycle::State & state)
{
  (void)state;
  reference_joint_state_publisher_->on_deactivate();
  return SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn InitControl::
on_shutdown(
  const rclcpp_lifecycle::State & state)
{
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn result = SUCCESS;
  switch (state.id()) {
    case lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE:
      result = this->on_deactivate(get_current_state());
      if (result != SUCCESS) {
        break;
      }
      result = this->on_cleanup(get_current_state());
      break;
    case lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE:
      result = this->on_cleanup(get_current_state());
      break;
    case lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED:
      break;
    default:
      break;
  }
  return result;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn InitControl::on_error(
  const rclcpp_lifecycle::State & state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "An error occured");
  return SUCCESS;
}

rcl_interfaces::msg::SetParametersResult InitControl::onParamChange(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = false;
  for (const rclcpp::Parameter & param : parameters) {
    if (param.get_name() == "lower_limits_deg" && canSetParameter(param)) {
      result.successful = onLowerLimitsChangeRequest(param);
    } else if (param.get_name() == "upper_limits_deg" && canSetParameter(param)) {
      result.successful = onUpperLimitsChangeRequest(param);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Invalid parameter name %s", param.get_name().c_str());
    }
  }
  return result;
}

bool InitControl::canSetParameter(const rclcpp::Parameter & param)
{
  try {
    if (!parameter_set_access_rights_.at(param.get_name()).isSetAllowed(
        this->get_current_state().id()))
    {
      RCLCPP_ERROR(
        this->get_logger(), "Parameter %s cannot be changed while in state %s",
        param.get_name().c_str(), this->get_current_state().label().c_str());
      return false;
    }
  } catch (const std::out_of_range & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Parameter set access rights for parameter %s couldn't be determined",
      param.get_name().c_str());
    return false;
  }
  return true;
}

bool InitControl::onLowerLimitsChangeRequest(const rclcpp::Parameter & param)
{
  if (param.get_type() != rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY) {
    RCLCPP_ERROR(
      this->get_logger(), "Invalid parameter type for parameter %s",
      param.get_name().c_str());
    return false;
  }


  if (param.as_double_array().size() != 7) {
    RCLCPP_ERROR(
      this->get_logger(), "Invalid parameter array length for parameter %s",
      param.get_name().c_str());
    return false;
  }
  std::transform(
    param.as_double_array().begin(),
    param.as_double_array().end(), lower_limits_rad_.begin(), d2r);
  return true;
}

bool InitControl::onUpperLimitsChangeRequest(const rclcpp::Parameter & param)
{
  if (param.get_type() != rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY) {
    RCLCPP_ERROR(
      this->get_logger(), "Invalid parameter type for parameter %s",
      param.get_name().c_str());
    return false;
  }
  if (param.as_double_array().size() != 7) {
    RCLCPP_ERROR(
      this->get_logger(), "Invalid parameter array length for parameter %s",
      param.get_name().c_str());
    return false;
  }
  std::transform(
    param.as_double_array().begin(),
    param.as_double_array().end(), upper_limits_rad_.begin(), d2r);
  return true;
}


}  // namespace driver_guided_robot


int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  auto node = std::make_shared<driver_guided_robot::InitControl>(
    "init_control",
    rclcpp::NodeOptions());
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
