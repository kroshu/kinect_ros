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

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_msgs/msg/bool.hpp"
#include "kuka_sunrise_interfaces/srv/get_state.hpp"
#include "kuka_sunrise/internal/service_tools.hpp"

namespace filter_points
{

class FilterPoints : public rclcpp::Node
{
public:
  FilterPoints(
    const std::string & node_name,
    const rclcpp::NodeOptions & options);

private:
  bool start_ = true;
  bool stop_ = false;
  bool fake_execution_;
  int lbr_state_ = 0;
  geometry_msgs::msg::Pose pelvis_pose_, handtip_pose_, wrist_pose_, thumb_pose_, rel_pose_,
    prev_rel_pose_, stop_pose_;
  geometry_msgs::msg::Point orientation_x_, orientation_y_, orientation_z_, left_stop_;
  rclcpp::Client<kuka_sunrise_interfaces::srv::GetState>::SharedPtr get_state_client_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr marker_listener_;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr goal_pos_publisher_;
  rclcpp::callback_group::CallbackGroup::SharedPtr cbg_;
  rclcpp::QoS qos_;
  void CoordinateTransform(geometry_msgs::msg::Point & pos1);
  geometry_msgs::msg::Point CrossProduct(
    const geometry_msgs::msg::Point & pos1,
    const geometry_msgs::msg::Point & pos2, bool normalize = true);
  geometry_msgs::msg::Point PoseDiff(
    const geometry_msgs::msg::Point & pos1,
    const geometry_msgs::msg::Point & pos2, bool coord_trans = true);
  geometry_msgs::msg::Quaternion ToQuaternion(
    const geometry_msgs::msg::Point & or1,
    const geometry_msgs::msg::Point & or2, const geometry_msgs::msg::Point & or3);
  void ChangeState(const std::string & node_name, std::uint8_t transition);
  void GetFRIState();
  void markersReceivedCallback(
    visualization_msgs::msg::MarkerArray::SharedPtr msg);
  enum class BODY_TRACKING_JOINTS
  {
    PELVIS = 0,
    SPINE_NAVEL,
    SPINE_CHEST,
    NECK,
    CLAVICLE_LEFT,
    SHOULDER_LEFT,
    ELBOW_LEFT,
    WRIST_LEFT,
    HAND_LEFT,
    HANDTIP_LEFT,
    THUMB_LEFT,
    CLAVICLE_RIGHT,
    SHOULDER_RIGHT,
    ELBOW_RIGHT,
    WRIST_RIGHT,
    HAND_RIGHT,
    HANDTIP_RIGHT,
    THUMB_RIGHT,
    HIP_LEFT,
    KNEE_LEFT,
    ANKLE_LEFT,
    FOOT_LEFT,
    HIP_RIGHT,
    KNEE_RIGHT,
    ANKLE_RIGHT,
    FOOT_RIGHT,
    HEAD,
    NOSE,
    EYE_LEFT,
    EAR_LEFT,
    EYE_RIGHT,
    EAR_RIGHT,
    COUNT
  };
};

}  // namespace filter_points

#endif  // FILTER_POINTS__FILTERPOINTS_HPP_
