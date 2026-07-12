# Advanced Topics Assignment 2 - Drone Mapper

Contributors: TODO - add names and IDs before submission.

## Build

```bash
cmake --preset default
cmake --build --preset default
```

The build creates:

```text
build/drone_mapper_simulation
build/maps_comparison
build/drone_mapper_simulation_test
```

The project uses C++20, `mp-units`, `tinynpy`, `yaml-cpp 0.9.0`, and GTest/GMock through the provided vcpkg/CMake setup. `yaml-cpp` is used for assignment YAML input parsing. GTest/GMock are used only by the test executable.

## Project Layout

Public Assignment 2 interfaces and required implementation headers stay under:

```text
include/drone_mapper/
```

Implementation files are grouped by component:

```text
src/config/        YAML composition and config loading
src/comparison/    map scoring utility and maps_comparison entry point
src/drone/         drone control
src/gps/           mock GPS simulation
src/lidar/         mock LiDAR implementation
src/main/          small simulator main entry point
src/map/           map storage and NPY I/O
src/movement/      mock movement simulation
src/mission/       mission loop and mapping algorithm
src/scan_result_to_voxels/
                   scan observation conversion into output-map voxels
src/simulation/    manager, run factory, and single-run orchestration
```

Tests keep the required Assignment 2 structure:

```text
tests/components/
tests/integration/
```

## Run Simulator

```bash
./build/drone_mapper_simulation [<simulation.yaml>] [<output_path>]
```

If the simulation composition path is missing, the executable uses `simulation.yaml` from the current working directory. Relative paths are resolved from the current working directory. Paths inside the composition file are resolved relative to the composition file directory.

Example:

```bash
./build/drone_mapper_simulation inputs/sim_compose.yaml /tmp/drone_mapper_output
```

The simulator writes:

```text
<output_path>/simulation_output.yaml
<output_path>/output_results/run_000/output_map.npy
<output_path>/output_results/run_001/output_map.npy
...
```

Each run directory may also contain `errors.log`. Errors are appended when they occur. Recoverable run failures receive score `-1` and the manager continues later runs.

## Run Maps Comparison

```bash
./build/maps_comparison <origin_map> <target_map> [comparison_config=<path>]
```

On success, stdout contains only the numeric score in the range `0..100`. On error, stdout contains `-1` and stderr contains the error message.

Without `comparison_config`, both maps are assumed to share shape, offset, bounds, and resolution. The optional config accepts the Assignment 2 rich format with `comparison_config.original` and `comparison_config.target`, each containing `map_res_cm`, `map_offset`, and `map_boundaries`. The older shared keys `map_resolution_cm`, `map_axes_offset`, and `boundaries` are still accepted for local tests.

## Run Tests

```bash
./build/drone_mapper_simulation_test
./build/drone_mapper_simulation_test --gtest_filter=Integration.*
./build/drone_mapper_simulation_test --gtest_filter=SimulationManager.*
./build/drone_mapper_simulation_test --gtest_filter=SimulationRun.*
./build/drone_mapper_simulation_test --gtest_filter=MissionControl.*
./build/drone_mapper_simulation_test --gtest_filter=DroneControl.*
./build/drone_mapper_simulation_test --gtest_filter=MappingAlgorithm.*
./build/drone_mapper_simulation_test --gtest_filter=MockLidar.*
./build/drone_mapper_simulation_test --gtest_filter=MapsComparison.*
```

Component tests are under `tests/components/`. Integration tests are under `tests/integration/`.

## Input Formats

The simulator reads the Assignment 2 YAML formats:

- composition YAML with simulation groups, mission config paths, drone config paths, and LiDAR config paths
- simulation YAML with `.npy` map path, map resolution, map axes offset, initial drone position, and initial angle
- mission YAML with max steps, mission bounds, GPS resolution, and optional `output_mapping_resolution_factor`
- drone YAML with `dimensions_cm`, max rotate, max advance, and max elevate
- LiDAR YAML with `z_min_cm`, `z_max_cm`, `d_cm`, and `fov_circles`

Drone `dimensions_cm` is converted to radius internally.

## Output And Scoring

`simulation_output.yaml` contains:

- composition file path
- generation timestamp
- metric name
- score range and error score
- total/scored/error run summary
- nested simulation config groups
- nested mission config groups with actual output resolution and resolution request status
- per-run drone config path, LiDAR config path, status, step count, score, and optional `error_ref`

Output maps are signed 8-bit `.npy` files using the assignment values:

```text
-3 PotentiallyOccupied
-2 OutOfBounds
-1 Unmapped
 0 Empty
 1 Occupied
```

## Implementation Notes

The map backend applies `map_axes_offset` before converting physical centimeters to array indices. Map bounds are treated as half-open physical ranges: `min <= position < max`.

`MappingAlgorithmImpl` keeps the required Assignment 2 `IMappingAlgorithm::nextStep(state, latest_scan)` API while reusing the Assignment 1 exploration idea: scan the current cell, read the output map that `DroneControlImpl` updates through `ScanResultToVoxels`, choose known-safe neighboring/frontier cells, and split rotate/advance/elevate commands according to the drone limits. When scanning is needed, it starts with the six principal directions, then aims additional rays at unresolved voxels that block otherwise-viable neighboring positions. Movement safety is checked again after every scan so the drone moves as soon as an unvisited path becomes known-safe. The search and adaptive scans are capped so all configuration combinations remain practical to run.

Resolution policy: missing `output_mapping_resolution_factor` defaults to `1`. Values below `1` are logged immediately, reported as `IGNORED_TOO_SMALL`, and run with factor `1`. Values `>= 1` are reported as `ACCEPTED`.

Mock movement can optionally validate movement against the hidden map in simulation runs while keeping the original `IDroneMovement` interface unchanged. The mapping algorithm is still intentionally bounded; it does not try to guarantee full-map coverage or return-to-start behavior.
