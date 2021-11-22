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

#ifndef MAP_ARM__MAPARM_HPP_
#define MAP_ARM__MAPARM_HPP_

#include <tf2_eigen/tf2_eigen.h>

#include <cinttypes>
#include <vector>
#include <string>
#include <map>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "utils/GeometryUtils.hpp"


namespace filter_points
{

class MapArm : public rclcpp::Node
{
public:
  MapArm(const std::string & node_name, const rclcpp::NodeOptions & options);

private:
  bool valid_ = true;
  bool motion_started_ = false;
  std::vector<double> prev_joint_state_;
  std::vector<int64_t> moving_avg_depth_;  // int is not supported for vectors, only uint8_t or long
  geometry_msgs::msg::Pose shoulder_pose_, rel_pose_, stop_pose_, prev_rel_pose_;
  geometry_msgs::msg::Point orientation_x_, orientation_y_, orientation_z_,
    left_stop_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr change_state_client_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr manage_processing_service_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr marker_listener_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr reference_publisher_;
  std_srvs::srv::Trigger::Request::SharedPtr trigger_request_ =
    std::make_shared<std_srvs::srv::Trigger::Request>();
  rclcpp::QoS qos_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_;
  bool onMovingAvgChangeRequest(const rclcpp::Parameter & param);
  rcl_interfaces::msg::SetParametersResult onParamChange(
    const std::vector<rclcpp::Parameter> & parameters);
  void markersReceivedCallback(
    visualization_msgs::msg::MarkerArray::SharedPtr msg);
};

}  // namespace filter_points

#endif  // MAP_ARM__MAPARM_HPP_
