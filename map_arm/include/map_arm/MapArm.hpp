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
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "map_arm/GeometryUtils.hpp"


namespace map_arm
{

class MapArm : public rclcpp::Node
{
public:
  MapArm(const std::string & node_name, const rclcpp::NodeOptions & options);

private:
  bool valid_;
  std::vector<double> prev_joint_state_;
  geometry_msgs::msg::Pose shoulder_pose_, elbow_pose, handtip_pose_,
    wrist_pose_, thumb_pose_, rel_pose_, stop_pose_;
  geometry_msgs::msg::Point orientation_x_, orientation_y_, orientation_z_,
    left_stop_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr change_state_client_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr manage_processing_service_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr marker_listener_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr reference_publisher_;
  std_srvs::srv::Trigger::Request::SharedPtr trigger_request_ =
    std::make_shared<std_srvs::srv::Trigger::Request>();
  rclcpp::QoS qos_;
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

}  // namespace map_arm

#endif  // MAP_ARM__MAPARM_HPP_
