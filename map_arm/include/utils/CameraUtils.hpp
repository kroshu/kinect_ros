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

#ifndef UTILS__CAMERAUTILS_HPP_
#define UTILS__CAMERAUTILS_HPP_

#include <cinttypes>
#include <vector>
#include <string>
#include <memory>

#include "Eigen/Geometry"

#include "boost/preprocessor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#define PROCESS_ONE_ELEMENT(r, unused, idx, elem) \
  BOOST_PP_COMMA_IF(idx) BOOST_PP_STRINGIZE(elem)

#define ENUM_MACRO(name, function, ...) \
  enum class name { __VA_ARGS__ }; \
  const char * name ## Strings[] = {BOOST_PP_SEQ_FOR_EACH_I( \
      PROCESS_ONE_ELEMENT, % %, BOOST_PP_VARIADIC_TO_SEQ( \
        __VA_ARGS__))}; \
  template<typename T> \
  constexpr const char * function ## ToString(T value) \
  {return name ## Strings[static_cast<int>(value)];}

namespace filter_points
{
ENUM_MACRO(
  BODY_TRACKING_JOINTS,
  Joint,
  PELVIS,
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
  COUNT)

geometry_msgs::msg::Vector3 operator+(
  const geometry_msgs::msg::Vector3 & v1,
  const geometry_msgs::msg::Vector3 & v2)
{
  geometry_msgs::msg::Vector3 sum;
  sum.x = v1.x + v2.x;
  sum.y = v1.y + v2.y;
  sum.z = v1.z + v2.z;
  return sum;
}

geometry_msgs::msg::Vector3 & operator+=(
  geometry_msgs::msg::Vector3 & v1,
  const geometry_msgs::msg::Vector3 & v2)
{
  v1 = v1 + v2;
  return v1;
}

geometry_msgs::msg::Vector3 operator/(const geometry_msgs::msg::Vector3 & v1, const double & factor)
{
  geometry_msgs::msg::Vector3 result;
  result.x = v1.x / factor;
  result.y = v1.y / factor;
  result.z = v1.z / factor;
  return result;
}

geometry_msgs::msg::Vector3 & operator/=(
  geometry_msgs::msg::Vector3 & v1,
  const double & factor)
{
  v1 = v1 / factor;
  return v1;
}

geometry_msgs::msg::Vector3 operator*(const geometry_msgs::msg::Vector3 & v1, const double & factor)
{
  geometry_msgs::msg::Vector3 result;
  result.x = v1.x * factor;
  result.y = v1.y * factor;
  result.z = v1.z * factor;
  return result;
}

geometry_msgs::msg::Vector3 & operator*=(
  geometry_msgs::msg::Vector3 & v1,
  const double & factor)
{
  v1 = v1 * factor;
  return v1;
}

geometry_msgs::msg::Point operator-(
  const geometry_msgs::msg::Point & p1,
  const geometry_msgs::msg::Point & p2)
{
  geometry_msgs::msg::Point result;
  result.x = p1.x - p2.x;
  result.y = p1.y - p2.y;
  result.z = p1.z - p2.z;
  return result;
}

/*
 *  @brief  Transforms the given point from the coordinate system of the camera
 *          to that of the robot
 *  @param x_angle and y_angle are the required angle corrections in the IMU coordinate system
 */
void cameraToRobot(
  geometry_msgs::msg::Point & pos1, const double & x_angle,
  const double & y_angle)
{
  Eigen::AngleAxisd rot1(-6 * M_PI / 180 + y_angle, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd rot2(M_PI / 2 - x_angle, Eigen::Vector3d::UnitZ());
  Eigen::AngleAxisd rot3(-M_PI / 2, Eigen::Vector3d::UnitY());
  Eigen::Matrix3d rot = rot3.toRotationMatrix() * rot2.toRotationMatrix() * rot1.toRotationMatrix();

  Eigen::Vector3d vector(pos1.x, pos1.y, pos1.z);
  Eigen::Vector3d vector_rot = rot * vector;

  pos1.x = vector_rot[0];
  pos1.y = vector_rot[1];
  pos1.z = vector_rot[2];
}

}  // namespace filter_points

#endif  // UTILS__CAMERAUTILS_HPP_
