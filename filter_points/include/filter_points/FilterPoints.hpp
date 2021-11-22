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

#ifndef FILTER_POINTS__FILTERPOINTS_HPP_
#define FILTER_POINTS__FILTERPOINTS_HPP_

#include <cinttypes>
#include <vector>
#include <string>
#include <map>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "utils/GeometryUtils.hpp"

namespace filter_points
{

class FilterPoints : public rclcpp::Node
{
public:
  FilterPoints(
    const std::string & node_name,
    const rclcpp::NodeOptions & options);

private:
  bool valid_;
  geometry_msgs::msg::Pose pelvis_pose_, handtip_pose_, wrist_pose_, thumb_pose_, rel_pose_,
    prev_rel_pose_, stop_pose_;
  geometry_msgs::msg::Point orientation_x_, orientation_y_, orientation_z_, left_stop_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr change_state_client_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr manage_processing_service_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr marker_listener_;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr goal_pos_publisher_;
  std_srvs::srv::Trigger::Request::SharedPtr trigger_request_ =
    std::make_shared<std_srvs::srv::Trigger::Request>();
  rclcpp::QoS qos_;
  void markersReceivedCallback(
    visualization_msgs::msg::MarkerArray::SharedPtr msg);
};

}  // namespace filter_points

#endif  // FILTER_POINTS__FILTERPOINTS_HPP_
