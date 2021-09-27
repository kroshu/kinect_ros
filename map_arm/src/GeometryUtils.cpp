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

#include <memory>
#include "map_arm/GeometryUtils.hpp"

namespace map_arm
{

void CameraToRobot(geometry_msgs::msg::Point & pos1)
{
  float tmp = pos1.x;
  pos1.x = -pos1.z;
  pos1.z = -pos1.y;
  pos1.y = tmp;
}

geometry_msgs::msg::Quaternion ToQuaternion(
  const geometry_msgs::msg::Point & or1,
  const geometry_msgs::msg::Point & or2,
  const geometry_msgs::msg::Point & or3)
{
  geometry_msgs::msg::Quaternion result;
  if (or1.x + or2.y + or3.z > 0) {
    result.w = sqrt(1 + or1.x + or2.y + or3.z) / 2;
    result.x = (or2.z - or3.y) / (4 * result.w);
    result.y = (or3.x - or1.z) / (4 * result.w);
    result.z = (or1.y - or2.x) / (4 * result.w);
  } else if (or1.x > or2.y && or1.x > or3.z) {
    float s = 2 * sqrt(1 + or1.x - or2.y - or3.z);
    result.w = (or2.z - or3.y) / s;
    result.x = s / 4;
    result.y = (or1.y + or2.x) / s;
    result.z = (or3.x + or1.z) / s;
  } else if (or2.y > or3.z) {
    float s = 2 * sqrt(1 - or1.x + or2.y - or3.z);
    result.w = (or3.x - or1.z) / s;
    result.x = (or1.y + or2.x) / s;
    result.y = s / 4;
    result.z = (or2.z + or3.y) / s;
  } else {
    float s = 2 * sqrt(1 - or1.x - or2.y + or3.z);
    result.w = (or1.y - or2.x) / s;
    result.x = (or3.x + or1.z) / s;
    result.y = (or2.z + or3.y) / s;
    result.z = s / 4;
  }
  float length = sqrt(
    result.x * result.x + result.y * result.y + result.z * result.z +
    result.w * result.w);
  result.x /= length;
  result.y /= length;
  result.z /= length;
  result.w /= length;

  return result;
}

geometry_msgs::msg::Point PoseDiff(
  const geometry_msgs::msg::Point & pos1,
  const geometry_msgs::msg::Point & pos2, bool coord_trans)
{
  geometry_msgs::msg::Point result;
  result.x = pos1.x - pos2.x;
  result.y = pos1.y - pos2.y;
  result.z = pos1.z - pos2.z;

  if (coord_trans) {
    CameraToRobot(result);
  }

  return result;
}

geometry_msgs::msg::Point CrossProduct(
  const geometry_msgs::msg::Point & pos1,
  const geometry_msgs::msg::Point & pos2, bool normalize)
{
  geometry_msgs::msg::Point result;
  result.x = pos1.y * pos2.z - pos1.z * pos2.y;
  result.y = -pos1.x * pos2.z + pos1.z * pos2.x;
  result.z = pos1.x * pos2.y - pos1.y * pos2.x;

  if (normalize) {
    float length = sqrt(
      result.x * result.x + result.y * result.y +
      result.z * result.z);
    result.x /= length;
    result.y /= length;
    result.z /= length;
  }
  return result;
}

}  // namespace map_arm
