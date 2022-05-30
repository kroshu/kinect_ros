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

#ifndef UTILS__GEOMETRYUTILS_HPP_
#define UTILS__GEOMETRYUTILS_HPP_

#include <cinttypes>
#include <vector>
#include <string>

#include <Eigen/Geometry>

#include "boost/preprocessor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#define PROCESS_ONE_ELEMENT(r, unused, idx, elem) \
  BOOST_PP_COMMA_IF(idx) BOOST_PP_STRINGIZE(elem)

#define ENUM_MACRO(name, ...) \
  enum class name { __VA_ARGS__ }; \
  const char * name ## Strings[] = {BOOST_PP_SEQ_FOR_EACH_I( \
      PROCESS_ONE_ELEMENT, % %, BOOST_PP_VARIADIC_TO_SEQ( \
        __VA_ARGS__))}; \
  template<typename T> \
  constexpr const char * name ## ToString(T value) \
  {return name ## Strings[static_cast<int>(value)];}

ENUM_MACRO(
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


namespace filter_points
{
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
 */
void cameraToRobot(geometry_msgs::msg::Point & pos1);

/*
 *  @brief  Transforms the given point from the coordinate system of the camera
 *          to that of the robot
 */
void cameraToRobotMod(geometry_msgs::msg::Point & pos1, const double & x_angle, const double & y_angle);

/*
 *  @brief  Calculates the crossproduct of 2 vectors
 *  @param  normalize (bool): if the result should be normalized
 */
geometry_msgs::msg::Point crossProduct(
  const geometry_msgs::msg::Point & pos1,
  const geometry_msgs::msg::Point & pos2, bool normalize = true);

/*
 *  @brief  Calculate pose difference
 *  @param  coord_trans (bool): if the result shoud be transformed
 *          from camera to robot frame
 */
geometry_msgs::msg::Point poseDiff(
  const geometry_msgs::msg::Point & pos1,
  const geometry_msgs::msg::Point & pos2, bool coord_trans = true);

/*
 *  @brief  Calculates the quaternion corresponding to the 3 unit vectors
 *  @param  or1, or2 and or3 are the unit vectors for x, y and z directions
 *          in the new coordinate system
 */
geometry_msgs::msg::Quaternion toQuaternion(
  const geometry_msgs::msg::Point & or1,
  const geometry_msgs::msg::Point & or2, const geometry_msgs::msg::Point & or3);

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

}  // namespace filter_points

#endif  // UTILS__GEOMETRYUTILS_HPP_
