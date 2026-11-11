// Copyright 2026 Tier IV, Inc.
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

// a static area must be a BOUNDING_BOX with positive dimensions
bool is_valid_static_area(const autoware_simulation_msgs::msg::SimulatedObject & static_area);

class StaticAreaRegistry
{
public:
  // DELETE_ALL removes all static areas, ADD and MODIFY replace the static area with the same id,
  // and any other action removes it.
  // ADD and MODIFY throw std::invalid_argument unless the static area is valid and in the map
  // frame.
  void apply(const autoware_simulation_msgs::msg::SimulatedObject & msg);

  [[nodiscard]] const std::vector<autoware_simulation_msgs::msg::SimulatedObject> & static_areas()
    const;

private:
  std::vector<autoware_simulation_msgs::msg::SimulatedObject> static_areas_;
};

}  // namespace autoware::dummy_perception_publisher

#endif  // AUTOWARE__DUMMY_PERCEPTION_PUBLISHER__STATIC_AREA_REGISTRY_HPP_
