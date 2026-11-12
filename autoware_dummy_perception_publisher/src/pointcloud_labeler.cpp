// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "autoware/dummy_perception_publisher/pointcloud_labeler.hpp"

#include <autoware/object_recognition_utils/pointcloud_classification.hpp>
#include <autoware/point_types/types.hpp>

#include <autoware_perception_msgs/msg/object_classification.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace autoware::dummy_perception_publisher
{

namespace
{
using autoware::point_types::PointXYZCPE;
using autoware::point_types::PointXYZIRC;

uint8_t to_pointcloud_classification_id(
  const autoware_perception_msgs::msg::ObjectClassification & classification)
{
  using autoware::point_types::PointCloudClassification;

  const auto pointcloud_classification =
    autoware::object_recognition_utils::try_into_pointcloud(classification.label);
  return static_cast<uint8_t>(
    pointcloud_classification.value_or(PointCloudClassification::INVALID));
}

pcl::PointCloud<PointXYZIRC> to_xyzirc(
  const pcl::PointCloud<pcl::PointXYZ> & input_cloud, const uint8_t intensity)
{
  pcl::PointCloud<PointXYZIRC> cloud_xyzirc;
  cloud_xyzirc.reserve(input_cloud.size());

  for (const auto & pt_xyz : input_cloud.points) {
    PointXYZIRC p_xyzirc;
    p_xyzirc.x = pt_xyz.x;
    p_xyzirc.y = pt_xyz.y;
    p_xyzirc.z = pt_xyz.z;
    p_xyzirc.intensity = intensity;
    p_xyzirc.return_type = 0;
    p_xyzirc.channel = 0;
    cloud_xyzirc.push_back(p_xyzirc);
  }

  return cloud_xyzirc;
}

pcl::PointCloud<PointXYZCPE> to_xyzcpe(
  const pcl::PointCloud<pcl::PointXYZ> & input_cloud, const uint8_t class_id,
  const float probability)
{
  pcl::PointCloud<PointXYZCPE> cloud_xyzcpe;
  cloud_xyzcpe.reserve(input_cloud.size());

  for (const auto & pt_xyz : input_cloud.points) {
    PointXYZCPE p_xyzcpe;
    p_xyzcpe.x = pt_xyz.x;
    p_xyzcpe.y = pt_xyz.y;
    p_xyzcpe.z = pt_xyz.z;
    p_xyzcpe.class_id = class_id;
    p_xyzcpe.probability = probability;
    p_xyzcpe.entropy = 0.0F;
    cloud_xyzcpe.push_back(p_xyzcpe);
  }

  return cloud_xyzcpe;
}
}  // namespace

PointType parse_point_type(const std::string & point_type)
{
  if (point_type == "XYZIRC") {
    return PointType::XYZIRC;
  }
  if (point_type == "XYZCPE") {
    return PointType::XYZCPE;
  }
  throw std::invalid_argument("point_type must be XYZIRC or XYZCPE");
}

PointCloudLabeler::PointCloudLabeler(const PointCloudLabelerConfig & config) : config_(config)
{
}

sensor_msgs::msg::PointCloud2 PointCloudLabeler::label(
  const std::vector<autoware_simulation_msgs::msg::SimulatedObject> & objects,
  const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> & object_pointclouds,
  const std::vector<autoware_simulation_msgs::msg::SimulatedObject> & static_areas,
  const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> & static_area_pointclouds) const
{
  if (
    objects.size() != object_pointclouds.size() ||
    static_areas.size() != static_area_pointclouds.size()) {
    throw std::invalid_argument("each object and static area requires exactly one point cloud");
  }

  sensor_msgs::msg::PointCloud2 output_pointcloud_msg;

  if (config_.point_type == PointType::XYZIRC) {
    pcl::PointCloud<PointXYZIRC> output_pointcloud;
    if (config_.publish_object_pointcloud) {
      for (const auto & object_pointcloud : object_pointclouds) {
        output_pointcloud += to_xyzirc(*object_pointcloud, 0);
      }
    }
    for (size_t i = 0; i < static_areas.size(); ++i) {
      output_pointcloud +=
        to_xyzirc(*static_area_pointclouds.at(i), static_areas.at(i).classification.label);
    }
    pcl::toROSMsg(output_pointcloud, output_pointcloud_msg);
    return output_pointcloud_msg;
  }

  pcl::PointCloud<PointXYZCPE> output_pointcloud;
  if (config_.publish_object_pointcloud) {
    for (size_t i = 0; i < objects.size(); ++i) {
      const auto & classification = objects.at(i).classification;
      output_pointcloud += to_xyzcpe(
        *object_pointclouds.at(i), to_pointcloud_classification_id(classification),
        classification.probability);
    }
  }
  for (size_t i = 0; i < static_areas.size(); ++i) {
    const auto & classification = static_areas.at(i).classification;
    output_pointcloud +=
      to_xyzcpe(*static_area_pointclouds.at(i), classification.label, classification.probability);
  }
  pcl::toROSMsg(output_pointcloud, output_pointcloud_msg);
  return output_pointcloud_msg;
}

}  // namespace autoware::dummy_perception_publisher
