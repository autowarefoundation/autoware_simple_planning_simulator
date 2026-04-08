# testing_purpose_repository_for_autoware_development
This repository is used for tests in autoware development

## Ported Packages
These packages were ported from Autoware Universe.

| Package | Upstream path | Ported-from commit | Notes |
| --- | --- | --- | --- |
| autoware_simple_planning_simulator | autoware_universe/simulator/autoware_simple_planning_simulator | https://github.com/autowarefoundation/autoware_universe/commit/3fd8d24e03d3373fbe8d80b29a184b8418cf6d49 | Ported as-is |
| autoware_dummy_perception_publisher | autoware_universe/simulator/autoware_dummy_perception_publisher | https://github.com/autowarefoundation/autoware_universe/commit/6c3792b8167e013968d60f33585b9a45ee2352f2 | Ported as-is |
| autoware_learning_based_vehicle_model | autoware_universe/simulator/autoware_learning_based_vehicle_model | https://github.com/autowarefoundation/autoware_universe/commit/846e79ceb272c887529a3fe432d46c97fac466ad | Ported as-is |

### Porting Checklist
1. Identify the upstream package path and the exact commit to port.
2. Copy the package into this repository and adjust build files only if required.
3. Run tests (if available) and record any local adjustments.
4. Commit the port with the upstream link footer and update this table.

### Commit Message Template
```
port(<package-name>): from Autoware Universe

  - Ported-from: https://github.com/autowarefoundation/autoware_universe/commit/<SHA>
  - Ported-path: <path/in/upstream>
  - Ported-date: <YYYY-MM-DD>
```
