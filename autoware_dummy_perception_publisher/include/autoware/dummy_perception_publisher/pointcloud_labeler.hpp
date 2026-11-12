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

#ifndef AUTOWARE__DUMMY_PERCEPTION_PUBLISHER__POINTCLOUD_LABELER_HPP_
#define AUTOWARE__DUMMY_PERCEPTION_PUBLISHER__POINTCLOUD_LABELER_HPP_

#include <autoware_simulation_msgs/msg/simulated_object.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <string>
#include <vector>

namespace autoware::dummy_perception_publisher
{

enum class PointType { XYZIRC, XYZCPE };

/**
 * @brief Parse the `point_type` parameter string.
 *
 * @param point_type `"XYZIRC"` or `"XYZCPE"`.
 * @return The corresponding PointType.
 * @throws std::invalid_argument If `point_type` is neither `"XYZIRC"` nor `"XYZCPE"`.
 */
PointType parse_point_type(const std::string & point_type);

struct PointCloudLabelerConfig
{
  PointType point_type;
  bool publish_object_pointcloud;
};

class PointCloudLabeler
{
public:
  explicit PointCloudLabeler(const PointCloudLabelerConfig & config);

  /**
   * @brief Merge the generated points into a single cloud of the configured point type.
   *
   * How each point is labeled depends on the point type:
   * - `XYZCPE`: object labels are converted from `ObjectClassification` to
   *   `PointCloudClassification` (`INVALID` if there is no counterpart), while static area labels
   *   are used as `PointCloudClassification` values as they are. `probability` is copied from the
   *   classification.
   * - `XYZIRC`: object points get intensity 0, and static area points get their label as
   *   intensity.
   *
   * Object points are included only if `publish_object_pointcloud` is enabled.
   *
   * @param objects Objects whose points are given in `object_pointclouds`.
   * @param object_pointclouds Points of each object; `object_pointclouds[i]` belongs to
   *                           `objects[i]`.
   * @param static_areas Static areas whose points are given in `static_area_pointclouds`.
   * @param static_area_pointclouds Points of each static area; `static_area_pointclouds[i]`
   *                                belongs to `static_areas[i]`.
   * @return The merged point cloud. Its header is left empty.
   * @throws std::invalid_argument If the number of point clouds differs from the number of
   *                               objects or static areas.
   */
  [[nodiscard]] sensor_msgs::msg::PointCloud2 label(
    const std::vector<autoware_simulation_msgs::msg::SimulatedObject> & objects,
    const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> & object_pointclouds,
    const std::vector<autoware_simulation_msgs::msg::SimulatedObject> & static_areas,
    const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> & static_area_pointclouds) const;

private:
  PointCloudLabelerConfig config_;
};

}  // namespace autoware::dummy_perception_publisher

#endif  // AUTOWARE__DUMMY_PERCEPTION_PUBLISHER__POINTCLOUD_LABELER_HPP_
