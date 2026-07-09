Project Name: Drone Mapper Simulation - Assignment 2
Course: Advanced Topics in Programming (2026B)
Contributors: Amit Reif-Tagari 322698986

1. General Description
    This project simulates a batch of autonomous drones, each building a 3D occupancy map of an
    unknown environment. A hidden ground-truth map (a binary .npy voxel grid) stands in for the
    real world; the drone only "sees" it through a simulated LiDAR sensor. The batch is described
    by a simulation_compositions.yaml file that references drone/lidar/simulation/mission YAML
    configs; the program expands every (simulation, mission) x drone x lidar combination, runs
    each one to completion (or until it errors or exhausts its step budget), compares the drone's
    generated map against the hidden one, and writes a hierarchical score report plus one output
    map per combination.

2. Architecture Overview
    DroneControlImpl executes a single step: it asks IMappingAlgorithm what to do next (given the
    previous step's LiDAR scan, or null on the very first call), performs the requested movement
    (Advance/Rotate/Elevate/Hover) if any, then performs the requested scan if the algorithm asked
    for one, folding the result into the output map via ScanResultToVoxels. Movement always
    happens before the scan, so a scan requested alongside movement observes the drone's
    post-movement state.
    MappingAlgorithmImpl implements the exploration strategy: a 6-directional (+-X, +-Y, +-Z)
    frontier/BFS search that expands outward from every newly-reached cell, skipping voxels
    already known to be Occupied and never re-queuing a cell once visited. Because a single LiDAR
    scan only covers roughly a forward hemisphere around the direction it's aimed at (see
    MockLidar's beam-offset math), the algorithm spends the first 6 calls at any newly-reached
    cell doing a full look-around (one scan_orientation per axis direction) before committing to a
    frontier decision from that vantage point. Because a single movement command can only change
    one physical DOF at a time, reaching a diagonal target may then take several more steps:
    elevate to match height, rotate to face the target, then advance.
    MissionControlImpl drives the step loop for a single mission: it checks the drone is still
    within the output map's boundaries before every step, stops on a drone-reported error, and
    stops (successfully) once the mapping algorithm reports it has finished. It always saves the
    output map before returning, regardless of outcome.
    SimulationManager expands the composition into every combination and hands each to
    ISimulationRunFactory; a combination that throws while being constructed or run (e.g. an
    unreadable map file) is caught, logged immediately, and recorded as an error run scored -1
    instead of aborting the whole batch. SimulationRunFactoryImpl wires up the full dependency
    graph (maps, GPS, movement, LiDAR, mapping algorithm, drone/mission control) for one
    combination, including resolving the mission's requested output resolution factor.
    CompositionLoader parses the YAML composition tree; SimulationReportWriter writes the final
    hierarchical simulation_output.yaml.

3. Input File Formats
    Map Files (.npy):        Binary 3D voxel grids (0 = empty, 1 = occupied, 2 = potentially
                              occupied, 255 = unmapped), read with the bundled TinyNPY parser.
    Configuration files
    (all YAML):
        simulation_compositions.yaml  Top-level file: lists simulation_config + mission_configs
                                       groups, plus drone_configs and lidar_configs. Every
                                       referenced path (including a simulation's own
                                       map_filename) is resolved relative to this file's own
                                       directory, matching the provided inputs/ sample layout.
        drone_config.yaml             dimensions_cm (sphere diameter), max_rotate_deg,
                                       max_advance_cm, max_elevate_cm.
        lidar_config.yaml             z_min_cm, z_max_cm, d_cm, fov_circles.
        mission_config.yaml           max_steps, boundaries (x/y/height min_cm+max_cm),
                                       gps_resolution_cm, optional output_mapping_resolution_factor
                                       (integer >= 1; missing defaults to factor 1; a value < 1 is
                                       ignored with an immediate error log).
        simulation_config.yaml        map_filename, map_resolution_cm, initial_drone_position
                                       (x_cm/y_cm/height_cm), initial_angle_deg, map_axes_offset
                                       (x_offset/y_offset/height_offset).
        comparison_config.yaml        Only read by the standalone maps_comparison utility:
                                       optional per-map resolution/offset/boundaries.

4. Output File Format and Directory Structure
    simulation_output.yaml:  A hierarchical score report written under the requested output
                              directory: score_report -> summary (total/scored/error run counts,
                              average/min/max score) -> simulations -> missions (each with its
                              resolved output resolution and ResolutionRequestStatus) -> runs
                              (one per drone x lidar combination, with status/steps/score/
                              output_map_file, plus an error_ref if it failed).
    output_results/:          Each combination gets its own nested folder
                              (simulation_<i>/mission_<i>/drone_<i>/lidar_<i>/map_output.npy) so
                              output maps never overwrite each other across a batch.
    error_log.txt:            Appended to (in the process's working directory) immediately
                              whenever a mission ends in error (e.g. "Drone exited mission
                              boundaries"), a resolution request is rejected, or a whole
                              combination fails to construct/run -- so failures stay visible even
                              though the run itself still returns a structured result instead of
                              crashing.

5. Error Handling and Fault Isolation
    MissionControlImpl checks the drone's position against the output map's boundaries before
    every step and aborts the mission with an Error status if it would be outside them. A
    drone-reported step error likewise aborts with Error status. If the mapping algorithm never
    finishes within the configured step budget, the mission ends with a MaxSteps status instead of
    throwing. One level up, SimulationManager wraps each combination's construction and run in a
    try/catch: an exception (e.g. a missing map file) is logged immediately and turned into an
    error-status SimulationResult (score -1) rather than stopping the batch, so the rest of the
    combinations still run.

6. Known Limitations
    - The mapping algorithm only ever targets the 6 axis-aligned neighbors of a cell (never a
      diagonal). A voxel that is only reachable by embedding the drone in solid space (e.g. a wall
      directly at the drone's spawn point, shadowed on all sides) may occasionally remain resolved
      only as "PotentiallyOccupied" rather than a confirmed "Occupied".
    - Integration tests that use the real algorithm (tests/integration/test_integration_real_algorithm.cpp)
      load their maps from a data_maps/ directory relative to the working directory the test
      binary is run from (typically the build/ directory). Those .npy fixtures are provided
      separately for this exercise and are not committed to source control; make sure data_maps/
      exists next to wherever you invoke drone_mapper_simulation_test from.

7. External Libraries
    mp-units:      Strong types for physical-unit safety (distances in cm, angles in deg)
                   throughout all internal APIs and calculations.
    TinyNPY:       Lightweight binary parser/writer for the .npy voxel grid map files.
    yaml-cpp:      Parses every configuration file (composition, drone/lidar/mission/simulation,
                   and the optional maps_comparison comparison_config) and writes
                   simulation_output.yaml.
    GTest / GMock: Component and integration test suites.

8. Build Instructions
    The project uses CMake, Ninja and vcpkg. Requires the VCPKG_ROOT environment variable to
    point at a vcpkg installation. From the repository root:
        Configure:  cmake --preset default
        Build:      cmake --build build
    The executables (drone_mapper_simulation, maps_comparison, drone_mapper_simulation_test)
    are created directly under build/. All targets compile with -Wall -Wextra -Werror -pedantic.

9. Execution
    Run a batch of simulations from a composition file (missing argument defaults to
    "simulation.yaml" in the current directory; a filename-only or relative path resolves against
    the current directory, an absolute path is used as-is; output defaults to the current
    directory):
        ./build/drone_mapper_simulation [composition_file] [output_dir]
    This writes output_dir/simulation_output.yaml and output_dir/output_results/... (see
    section 4).

    Compare two .npy maps directly, optionally with a YAML comparison config:
        ./build/maps_comparison <origin_map.npy> <target_map.npy> [comparison_config=<path>]

    Run the full test suite (component + integration tests; see section 6 for the data_maps/
    requirement of the real-algorithm integration tests):
        ./build/drone_mapper_simulation_test
    Filter to one component, e.g.:
        ./build/drone_mapper_simulation_test --gtest_filter=MappingAlgorithm.*
        ./build/drone_mapper_simulation_test --gtest_filter=Integration.*

10. Bonus Features
    See bonus.txt for the implemented bonus feature (comparing maps with different resolutions
    in MapsComparison) and how to run its dedicated test.

11. Design Documentation
    See Additional_Docs.pdf in the repository root for the full high-level design (component
    responsibilities, class diagram, and sequence diagrams for the mission-run and exploration
    flows).
