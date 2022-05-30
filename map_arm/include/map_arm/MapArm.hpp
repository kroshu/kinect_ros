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
#include "sensor_msgs/msg/imu.hpp"
#include "camera_msgs/msg/marker_array.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "utils/CameraUtils.hpp"
#include "kuka_sunrise/internal/service_tools.hpp"
#include "kroshu_ros2_core/ROS2BaseNode.hpp"


namespace filter_points
{

class MapArm : public kroshu_ros2_core::ROS2BaseNode
{
public:
  MapArm(const std::string & node_name, const rclcpp::NodeOptions & options);
  ~MapArm() override;

private:
  bool valid_ = true;
  bool initialized_ = false;
  bool motion_started_ = false;
  bool record_ = false;
  int bag_count_ = 0;
  int imu_count_ = 0;
  std::vector<double> prev_joint_state_ = std::vector<double>(7);
  // int is not supported for vectors, only uint8_t or long
  std::vector<int64_t> moving_avg_depth_ = std::vector<int64_t>(7);
  geometry_msgs::msg::Point prev_rel_pos_;
  geometry_msgs::msg::Vector3 imu_acceleration_;
  double x_angle_;
  double y_angle_;
  std::unique_ptr<rosbag2_cpp::writers::SequentialWriter> rosbag_writer_;

  bool onMovingAvgChangeRequest(const std::vector<int64_t> & moving_avg);
  bool FindMarker(const camera_msgs::msg::Marker & marker, BODY_TRACKING_JOINTS joint);
  void writeBagFile(const sensor_msgs::msg::JointState & reference);
  void manageProcessingCallback(
    std_msgs::msg::Bool::SharedPtr valid);
  void markersReceivedCallback(camera_msgs::msg::MarkerArray::SharedPtr msg);
  void imuReceivedCallback(sensor_msgs::msg::Imu::SharedPtr msg);
  void calculateAngles();
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
  rclcpp::Subscription<camera_msgs::msg::MarkerArray>::SharedPtr marker_listener_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_listener_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr reference_publisher_;

  std_srvs::srv::Trigger::Request::SharedPtr trigger_request_ =
    std::make_shared<std_srvs::srv::Trigger::Request>();
  rclcpp::QoS qos_ = rclcpp::QoS(rclcpp::KeepLast(1));
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_;
  rclcpp::CallbackGroup::SharedPtr cbg_;
};
}  // namespace filter_points

#endif  // MAP_ARM__MAPARM_HPP_
