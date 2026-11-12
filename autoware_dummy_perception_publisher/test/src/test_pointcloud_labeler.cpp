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

#include <autoware/point_types/types.hpp>

#include <autoware_perception_msgs/msg/object_classification.hpp>
#include <autoware_simulation_msgs/msg/simulated_object.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <gtest/gtest.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cstdint>
#include <stdexcept>
#include <vector>

namespace autoware::dummy_perception_publisher
{
namespace
{
using autoware::point_types::PointXYZCPE;
using autoware::point_types::PointXYZIRC;
using autoware_perception_msgs::msg::ObjectClassification;
using autoware_simulation_msgs::msg::SimulatedObject;

// STRUCTURE in autoware::point_types::PointCloudClassification
constexpr uint8_t structure_class_id = 9;

pcl::PointCloud<pcl::PointXYZ>::Ptr create_single_point_cloud(
  const float x, const float y, const float z)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  cloud->push_back(pcl::PointXYZ(x, y, z));
  return cloud;
}

SimulatedObject create_labeled_object(const uint8_t label, const float probability)
{
  SimulatedObject object;
  object.classification.label = label;
  object.classification.probability = probability;
  return object;
}

PointCloudLabeler create_labeler(const PointType point_type, const bool publish_object_pointcloud)
{
  return PointCloudLabeler(PointCloudLabelerConfig{point_type, publish_object_pointcloud});
}

pcl::PointCloud<PointXYZCPE> to_xyzcpe_cloud(const sensor_msgs::msg::PointCloud2 & msg)
{
  pcl::PointCloud<PointXYZCPE> cloud;
  pcl::fromROSMsg(msg, cloud);
  return cloud;
}

pcl::PointCloud<PointXYZIRC> to_xyzirc_cloud(const sensor_msgs::msg::PointCloud2 & msg)
{
  pcl::PointCloud<PointXYZIRC> cloud;
  pcl::fromROSMsg(msg, cloud);
  return cloud;
}
}  // namespace

TEST(ParsePointTypeTest, ReturnsXYZIRCForXYZIRCString)
{
  EXPECT_EQ(parse_point_type("XYZIRC"), PointType::XYZIRC);
}

TEST(ParsePointTypeTest, ReturnsXYZCPEForXYZCPEString)
{
  EXPECT_EQ(parse_point_type("XYZCPE"), PointType::XYZCPE);
}

TEST(ParsePointTypeTest, ThrowsInvalidArgumentWhenPointTypeIsUnknown)
{
  EXPECT_THROW(parse_point_type("XYZI"), std::invalid_argument);
}

TEST(PointCloudLabelerTest, KeepsPointCoordinatesInXYZCPE)
{
  // Arrange
  const auto labeler = create_labeler(PointType::XYZCPE, false);
  const auto static_area = create_labeled_object(structure_class_id, 1.0F);
  const auto static_area_cloud = create_single_point_cloud(1.0F, 2.0F, 3.0F);

  // Act
  const auto output = to_xyzcpe_cloud(labeler.label({}, {}, {static_area}, {static_area_cloud}));

  // Assert
  ASSERT_EQ(output.size(), 1U);
  EXPECT_FLOAT_EQ(output.at(0).x, 1.0F);
  EXPECT_FLOAT_EQ(output.at(0).y, 2.0F);
  EXPECT_FLOAT_EQ(output.at(0).z, 3.0F);
}

TEST(PointCloudLabelerTest, ConvertsObjectLabelToPointCloudClassificationInXYZCPE)
{
  // Arrange
  const auto labeler = create_labeler(PointType::XYZCPE, true);
  const auto object = create_labeled_object(ObjectClassification::CAR, 0.8F);
  const auto object_cloud = create_single_point_cloud(1.0F, 0.0F, 0.0F);

  // Act
  const auto output = to_xyzcpe_cloud(labeler.label({object}, {object_cloud}, {}, {}));

  // Assert
  ASSERT_EQ(output.size(), 1U);
  EXPECT_EQ(output.at(0).class_id, 0U);  // PointCloudClassification::CAR
  EXPECT_FLOAT_EQ(output.at(0).probability, 0.8F);
}

TEST(PointCloudLabelerTest, SetsInvalidClassForObjectLabelWithoutPointCloudClassification)
{
  // Arrange
  const auto labeler = create_labeler(PointType::XYZCPE, true);
  const auto object = create_labeled_object(ObjectClassification::UNKNOWN, 0.8F);
  const auto object_cloud = create_single_point_cloud(1.0F, 0.0F, 0.0F);

  // Act
  const auto output = to_xyzcpe_cloud(labeler.label({object}, {object_cloud}, {}, {}));

  // Assert
  ASSERT_EQ(output.size(), 1U);
  EXPECT_EQ(output.at(0).class_id, 255U);  // PointCloudClassification::INVALID
}

TEST(PointCloudLabelerTest, UsesStaticAreaLabelAsPointCloudClassificationInXYZCPE)
{
  // Arrange
  const auto labeler = create_labeler(PointType::XYZCPE, false);
  const auto static_area = create_labeled_object(structure_class_id, 0.6F);
  const auto static_area_cloud = create_single_point_cloud(1.0F, 0.0F, 0.0F);

  // Act
  const auto output = to_xyzcpe_cloud(labeler.label({}, {}, {static_area}, {static_area_cloud}));

  // Assert
  ASSERT_EQ(output.size(), 1U);
  EXPECT_EQ(output.at(0).class_id, structure_class_id);
  EXPECT_FLOAT_EQ(output.at(0).probability, 0.6F);
}

TEST(PointCloudLabelerTest, ExcludesObjectPointsWhenPublishObjectPointcloudIsFalse)
{
  // Arrange
  const auto labeler = create_labeler(PointType::XYZCPE, false);
  const auto object = create_labeled_object(ObjectClassification::CAR, 1.0F);
  const auto object_cloud = create_single_point_cloud(1.0F, 0.0F, 0.0F);
  const auto static_area = create_labeled_object(structure_class_id, 1.0F);
  const auto static_area_cloud = create_single_point_cloud(2.0F, 0.0F, 0.0F);

  // Act
  const auto output =
    to_xyzcpe_cloud(labeler.label({object}, {object_cloud}, {static_area}, {static_area_cloud}));

  // Assert
  ASSERT_EQ(output.size(), 1U);
  EXPECT_FLOAT_EQ(output.at(0).x, 2.0F);
}

TEST(PointCloudLabelerTest, IncludesObjectPointsWhenPublishObjectPointcloudIsTrue)
{
  // Arrange
  const auto labeler = create_labeler(PointType::XYZCPE, true);
  const auto object = create_labeled_object(ObjectClassification::CAR, 1.0F);
  const auto object_cloud = create_single_point_cloud(1.0F, 0.0F, 0.0F);
  const auto static_area = create_labeled_object(structure_class_id, 1.0F);
  const auto static_area_cloud = create_single_point_cloud(2.0F, 0.0F, 0.0F);

  // Act
  const auto output =
    to_xyzcpe_cloud(labeler.label({object}, {object_cloud}, {static_area}, {static_area_cloud}));

  // Assert
  EXPECT_EQ(output.size(), 2U);
}

TEST(PointCloudLabelerTest, SetsObjectIntensityToZeroInXYZIRC)
{
  // Arrange
  const auto labeler = create_labeler(PointType::XYZIRC, true);
  const auto object = create_labeled_object(ObjectClassification::TRUCK, 1.0F);
  const auto object_cloud = create_single_point_cloud(1.0F, 0.0F, 0.0F);

  // Act
  const auto output = to_xyzirc_cloud(labeler.label({object}, {object_cloud}, {}, {}));

  // Assert
  ASSERT_EQ(output.size(), 1U);
  EXPECT_EQ(output.at(0).intensity, 0U);
}

TEST(PointCloudLabelerTest, SetsStaticAreaIntensityToItsLabelInXYZIRC)
{
  // Arrange
  const auto labeler = create_labeler(PointType::XYZIRC, false);
  const auto static_area = create_labeled_object(structure_class_id, 1.0F);
  const auto static_area_cloud = create_single_point_cloud(1.0F, 0.0F, 0.0F);

  // Act
  const auto output = to_xyzirc_cloud(labeler.label({}, {}, {static_area}, {static_area_cloud}));

  // Assert
  ASSERT_EQ(output.size(), 1U);
  EXPECT_EQ(output.at(0).intensity, structure_class_id);
}

TEST(PointCloudLabelerTest, ThrowsInvalidArgumentWhenPointCloudCountDiffersFromObjectCount)
{
  // Arrange
  const auto labeler = create_labeler(PointType::XYZCPE, true);
  const auto object = create_labeled_object(ObjectClassification::CAR, 1.0F);

  // Act & Assert
  EXPECT_THROW(static_cast<void>(labeler.label({object}, {}, {}, {})), std::invalid_argument);
}

}  // namespace autoware::dummy_perception_publisher
