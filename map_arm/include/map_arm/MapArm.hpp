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
#include <cstdio>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_cpp/writers/sequential_writer.hpp"
#include "rosbag2_storage/serialized_bag_message.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "utils/GeometryUtils.hpp"
#include "kuka_sunrise/internal/service_tools.hpp"


namespace filter_points
{

// TODO(kovacsge11) maybe derive from ROS2BaseNode
class MapArm : public rclcpp::Node
{
public:
  MapArm(const std::string & node_name, const rclcpp::NodeOptions & options);
  ~MapArm();

private:
  bool valid_ = true;
  bool motion_started_ = false;
  bool record_ = false;
  int bag_count_ = 0;
  std::vector<double> prev_joint_state_ = std::vector<double>(7);
  // int is not supported for vectors, only uint8_t or long
  std::vector<int64_t> moving_avg_depth_ = std::vector<int64_t>(7);
  geometry_msgs::msg::Point prev_rel_pos_;
  std::unique_ptr<rosbag2_cpp::writers::SequentialWriter> rosbag_writer_;

  rcl_interfaces::msg::SetParametersResult onParamChange(
    const std::vector<rclcpp::Parameter> & parameters);
  bool onMovingAvgChangeRequest(const rclcpp::Parameter & param);
  void writeBagFile(const sensor_msgs::msg::JointState & reference);
  void manageProcessingCallback(
    std_msgs::msg::Bool::SharedPtr valid);
  void markersReceivedCallback(
    visualization_msgs::msg::MarkerArray::SharedPtr msg);
  void calculateJoints12(
    std::vector<double> & joint_state,
    const geometry_msgs::msg::Point & elbow_rel_pos);
  void calculateJoints34(
    std::vector<double> & joint_state,
    const geometry_msgs::msg::Point & wrist_rel_pos);
  void calculateJoints56(
    std::vector<double> & joint_state,
    const geometry_msgs::msg::Point & handtip_rel_pos);

  const rosbag2_cpp::StorageOptions storage_options_ = rosbag2_cpp::StorageOptions(
    {"replay",
      "sqlite3"});

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr change_state_client_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr manage_processing_sub_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr marker_listener_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr reference_publisher_;

  std_srvs::srv::Trigger::Request::SharedPtr trigger_request_ =
    std::make_shared<std_srvs::srv::Trigger::Request>();
  rclcpp::QoS qos_ = rclcpp::QoS(rclcpp::KeepLast(1));
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_;
  rclcpp::CallbackGroup::SharedPtr cbg_;
};
}  // namespace filter_points

#endif  // MAP_ARM__MAPARM_HPP_
