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

#include "autoware/dummy_perception_publisher/static_area_registry.hpp"

#include <autoware_perception_msgs/msg/shape.hpp>
#include <autoware_simulation_msgs/msg/simulated_object.hpp>

#include <gtest/gtest.h>

#include <cstdint>
#include <stdexcept>

namespace autoware::dummy_perception_publisher
{
namespace
{
using autoware_perception_msgs::msg::Shape;
using autoware_simulation_msgs::msg::SimulatedObject;

// a valid 1 m cube static area in the map frame
SimulatedObject create_static_area(const uint8_t id, const uint8_t action)
{
  SimulatedObject static_area;
  static_area.header.frame_id = "map";
  static_area.id.uuid.at(0) = id;
  static_area.action = action;
  static_area.shape.type = Shape::BOUNDING_BOX;
  static_area.shape.dimensions.x = 1.0;
  static_area.shape.dimensions.y = 1.0;
  static_area.shape.dimensions.z = 1.0;
  return static_area;
}
}  // namespace

TEST(IsValidStaticAreaTest, ReturnsTrueForBoundingBoxWithPositiveDimensions)
{
  EXPECT_TRUE(is_valid_static_area(create_static_area(1, SimulatedObject::ADD)));
}

TEST(IsValidStaticAreaTest, ReturnsFalseWhenShapeIsNotBoundingBox)
{
  auto static_area = create_static_area(1, SimulatedObject::ADD);
  static_area.shape.type = Shape::CYLINDER;

  EXPECT_FALSE(is_valid_static_area(static_area));
}

TEST(IsValidStaticAreaTest, ReturnsFalseWhenDimensionIsZero)
{
  auto static_area = create_static_area(1, SimulatedObject::ADD);
  static_area.shape.dimensions.z = 0.0;

  EXPECT_FALSE(is_valid_static_area(static_area));
}

TEST(StaticAreaRegistryTest, StoresStaticAreaOnAdd)
{
  // Arrange
  StaticAreaRegistry registry;

  // Act
  registry.apply(create_static_area(1, SimulatedObject::ADD));

  // Assert
  ASSERT_EQ(registry.static_areas().size(), 1U);
  EXPECT_EQ(registry.static_areas().at(0).id.uuid.at(0), 1U);
}

TEST(StaticAreaRegistryTest, KeepsStaticAreasWithDifferentIds)
{
  // Arrange
  StaticAreaRegistry registry;
  registry.apply(create_static_area(1, SimulatedObject::ADD));

  // Act
  registry.apply(create_static_area(2, SimulatedObject::ADD));

  // Assert
  EXPECT_EQ(registry.static_areas().size(), 2U);
}

TEST(StaticAreaRegistryTest, ReplacesStaticAreaWithSameIdOnModify)
{
  // Arrange
  StaticAreaRegistry registry;
  registry.apply(create_static_area(1, SimulatedObject::ADD));
  auto modified_static_area = create_static_area(1, SimulatedObject::MODIFY);
  modified_static_area.shape.dimensions.x = 2.0;

  // Act
  registry.apply(modified_static_area);

  // Assert
  ASSERT_EQ(registry.static_areas().size(), 1U);
  EXPECT_DOUBLE_EQ(registry.static_areas().at(0).shape.dimensions.x, 2.0);
}

TEST(StaticAreaRegistryTest, RemovesOnlyStaticAreaWithSameIdOnDelete)
{
  // Arrange
  StaticAreaRegistry registry;
  registry.apply(create_static_area(1, SimulatedObject::ADD));
  registry.apply(create_static_area(2, SimulatedObject::ADD));

  // Act
  registry.apply(create_static_area(1, SimulatedObject::DELETE));

  // Assert
  ASSERT_EQ(registry.static_areas().size(), 1U);
  EXPECT_EQ(registry.static_areas().at(0).id.uuid.at(0), 2U);
}

TEST(StaticAreaRegistryTest, RemovesAllStaticAreasOnDeleteAll)
{
  // Arrange
  StaticAreaRegistry registry;
  registry.apply(create_static_area(1, SimulatedObject::ADD));
  registry.apply(create_static_area(2, SimulatedObject::ADD));

  // Act
  registry.apply(create_static_area(0, SimulatedObject::DELETE_ALL));

  // Assert
  EXPECT_TRUE(registry.static_areas().empty());
}

TEST(StaticAreaRegistryTest, ThrowsInvalidArgumentWhenAddedStaticAreaIsNotInMapFrame)
{
  // Arrange
  StaticAreaRegistry registry;
  auto static_area = create_static_area(1, SimulatedObject::ADD);
  static_area.header.frame_id = "base_link";

  // Act & Assert
  EXPECT_THROW(registry.apply(static_area), std::invalid_argument);
}

TEST(StaticAreaRegistryTest, ThrowsInvalidArgumentWhenAddedStaticAreaIsInvalid)
{
  // Arrange
  StaticAreaRegistry registry;
  auto static_area = create_static_area(1, SimulatedObject::ADD);
  static_area.shape.type = Shape::CYLINDER;

  // Act & Assert
  EXPECT_THROW(registry.apply(static_area), std::invalid_argument);
}

}  // namespace autoware::dummy_perception_publisher
