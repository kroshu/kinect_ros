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

#include <Eigen/Geometry>

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
  std::vector<double> prev_joint_state_ = std::vector<double>(7);
  // int is not supported for vectors, only uint8_t or long
  std::vector<int64_t> moving_avg_depth_ = std::vector<int64_t>(7);
  geometry_msgs::msg::Point prev_rel_pose_;

  rcl_interfaces::msg::SetParametersResult onParamChange(
    const std::vector<rclcpp::Parameter> & parameters);
  bool onMovingAvgChangeRequest(const rclcpp::Parameter & param);
  void markersReceivedCallback(
    visualization_msgs::msg::MarkerArray::SharedPtr msg);
  void calculateJoints12(
    std::vector<double> & joint_state,
    geometry_msgs::msg::Point elbow_rel_pos);
  void calculateJoints34(
    std::vector<double> & joint_state,
    geometry_msgs::msg::Point wrist_rel_pos);
  void calculateJoints56(
    std::vector<double> & joint_state,
    geometry_msgs::msg::Point handtip_rel_pos);

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr change_state_client_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr manage_processing_service_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr marker_listener_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr reference_publisher_;

  std_srvs::srv::Trigger::Request::SharedPtr trigger_request_ =
    std::make_shared<std_srvs::srv::Trigger::Request>();
  rclcpp::QoS qos_ = rclcpp::QoS(rclcpp::KeepLast(1));
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_;
};
}  // namespace filter_points

#endif  // MAP_ARM__MAPARM_HPP_
