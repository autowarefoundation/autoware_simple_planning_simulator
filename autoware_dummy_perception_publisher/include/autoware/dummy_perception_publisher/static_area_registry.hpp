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

#ifndef AUTOWARE__DUMMY_PERCEPTION_PUBLISHER__STATIC_AREA_REGISTRY_HPP_
#define AUTOWARE__DUMMY_PERCEPTION_PUBLISHER__STATIC_AREA_REGISTRY_HPP_

#include <autoware_simulation_msgs/msg/simulated_object.hpp>

#include <vector>

namespace autoware::dummy_perception_publisher
{

/**
 * @brief Check whether the given object can be used as a static area.
 *
 * @param static_area Static area to check.
 * @return True if its shape is `BOUNDING_BOX` with positive dimensions.
 */
bool is_valid_static_area(const autoware_simulation_msgs::msg::SimulatedObject & static_area);

class StaticAreaRegistry
{
public:
  /**
   * @brief Update the stored static areas according to the action of the given message.
   *
   * - `ADD` / `MODIFY`: store the static area, replacing any stored one with the same id.
   * - `DELETE`: remove the stored static area with the same id.
   * - `DELETE_ALL`: remove all stored static areas.
   * - Any other action: treated as `DELETE`.
   *
   * @param msg Static area to apply. For `ADD` / `MODIFY`, it must satisfy
   *            is_valid_static_area() and be in the `map` frame.
   * @throws std::invalid_argument If `ADD` / `MODIFY` is given an invalid static area or one
   *                               not in the `map` frame.
   */
  void apply(const autoware_simulation_msgs::msg::SimulatedObject & msg);

  /**
   * @brief Get the stored static areas.
   *
   * @return Stored static areas, all in the `map` frame.
   */
  [[nodiscard]] const std::vector<autoware_simulation_msgs::msg::SimulatedObject> & static_areas()
    const;

private:
  std::vector<autoware_simulation_msgs::msg::SimulatedObject> static_areas_;
};

}  // namespace autoware::dummy_perception_publisher

#endif  // AUTOWARE__DUMMY_PERCEPTION_PUBLISHER__STATIC_AREA_REGISTRY_HPP_
