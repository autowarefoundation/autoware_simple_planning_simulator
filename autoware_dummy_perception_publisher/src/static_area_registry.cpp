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

#include <algorithm>
#include <stdexcept>
#include <vector>

namespace autoware::dummy_perception_publisher
{
using autoware_simulation_msgs::msg::SimulatedObject;

bool is_valid_static_area(const SimulatedObject & static_area)
{
  const auto & shape = static_area.shape;
  return shape.type == autoware_perception_msgs::msg::Shape::BOUNDING_BOX &&
         shape.dimensions.x > 0.0 && shape.dimensions.y > 0.0 && shape.dimensions.z > 0.0;
}

void StaticAreaRegistry::apply(const SimulatedObject & msg)
{
  if (msg.action == SimulatedObject::DELETE_ALL) {
    static_areas_.clear();
    return;
  }

  const bool is_upsert =
    msg.action == SimulatedObject::ADD || msg.action == SimulatedObject::MODIFY;
  if (is_upsert && !is_valid_static_area(msg)) {
    throw std::invalid_argument("static area requires BOUNDING_BOX shape with positive dimensions");
  }
  if (is_upsert && msg.header.frame_id != "map") {
    throw std::invalid_argument("static area must be in the map frame");
  }

  static_areas_.erase(
    std::remove_if(
      static_areas_.begin(), static_areas_.end(),
      [&](const auto & static_area) { return static_area.id == msg.id; }),
    static_areas_.end());

  if (is_upsert) {
    static_areas_.push_back(msg);
  }
}

const std::vector<SimulatedObject> & StaticAreaRegistry::static_areas() const
{
  return static_areas_;
}

}  // namespace autoware::dummy_perception_publisher
