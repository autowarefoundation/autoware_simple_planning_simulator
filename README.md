# autoware_simple_planning_simulator
This repository contains packages for planning simulation in Autoware. It includes a simple planning simulator, a dummy perception publisher, and a learning-based vehicle model. These packages were ported from the Autoware Universe repository to provide a standalone simulation environment for testing and development.

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
  - Source-path: <path/in/upstream>
  - Ported-date: <YYYY-MM-DD>
```
