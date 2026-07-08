Project Name: Drone Mapper Simulation - Assignment 2
Course: Advanced Topics in Programming (2026B)
Contributors: Amit Reif-Tagari 322698986

1. General Description
    This project simulates an autonomous drone that builds a 3D occupancy map of an unknown
    environment. A hidden ground-truth map (a binary .npy voxel grid) stands in for the real
    world; the drone only "sees" it through a simulated LiDAR sensor. Each simulation run wires
    together a drone, a GPS, a movement actuator, a LiDAR, and a mapping algorithm around a
    mutable output map, then steps the drone forward until it either finishes exploring, exceeds
    its step budget, or violates a mission constraint (e.g. leaving the map boundaries).
    The resulting output map is compared voxel-by-voxel against the hidden map to produce a
    mapping accuracy score (0-100).

2. Architecture Overview
    DroneControlImpl orchestrates a single step: it scans with the LiDAR (in all 6 axis
    directions relative to the drone's heading, for full spherical coverage regardless of which
    way the drone happens to be facing), writes the observed occupancy into the output map via
    ScanResultToVoxels, asks the IMappingAlgorithm for the next move, and dispatches that move to
    IDroneMovement (Advance / Rotate / Elevate / Hover).
    MappingAlgorithmImpl implements the exploration strategy: a 6-directional (+-X, +-Y, +-Z)
    frontier/BFS search that expands outward from every newly-reached cell, skipping voxels
    already known to be Occupied and never re-queuing a cell once it has been visited. Because
    a single movement command can only change the drone's position along one physical DOF at a
    time (a rotation, or a straight advance along the current heading, or a vertical elevation),
    reaching a diagonal target may take several steps: elevate to match height, rotate to face
    the target, then advance.
    MissionControlImpl drives the step loop for a single mission: it checks the drone is still
    within the map boundaries before every step, stops on a drone-reported error, and stops
    (successfully) once the mapping algorithm reports it has finished.
    SimulationManager / SimulationRunFactoryImpl wire up the full dependency graph (maps, GPS,
    movement, LiDAR, mapping algorithm, drone/mission control) for the Cartesian product of the
    configured simulations x missions x drones x lidars, and hand back one SimulationResult per
    combination, including its final mapping score.

3. Input File Formats
    Map Files (.npy):   Binary files containing a 3D voxel grid (0 = empty, 1 = occupied,
                         2 = potentially occupied, 255 = unmapped), read with the bundled
                         lightweight TinyNPY parser. Used both as the hidden ground-truth map
                         and as the freshly-allocated output map that the drone fills in.
    YAML config:        Only the standalone maps_comparison utility currently reads a YAML file
                         (an optional "comparison_config" describing resolution/offset/boundaries
                         for the two maps being compared). The main simulation binary accepts a
                         composition-file path on the command line for forward compatibility, but
                         the current build runs a single hardcoded example scenario rather than
                         parsing combinations from that file (see section 6, Known Limitations).

4. Output File Format and Directory Structure
    map_output.npy:   The drone's own output map, written per simulation run as a binary .npy
                       file under the requested output directory, reflecting everything the
                       drone believes it observed (Empty / Occupied / PotentiallyOccupied /
                       Unmapped) for every voxel.
    error_log.txt:     Appended to (in the process's working directory) whenever a mission ends
                       in error, e.g. "Drone exited mission boundaries" or a drone-reported
                       failure, so failures remain visible even though the run itself still
                       returns a structured MissionRunResult/SimulationResult instead of crashing.

5. Error Handling and Fault Isolation
    MissionControlImpl checks the drone's position against the map boundaries before every step
    and aborts the mission with an Error status (logged to error_log.txt) if the drone would be
    outside them. A drone-reported step error likewise aborts with Error status. If the mapping
    algorithm never finishes within the configured step budget, the mission ends with a MaxSteps
    status instead of throwing. In all three cases a MissionRunResult/SimulationResult is still
    returned normally, so a single bad combination in a batch of simulations does not stop the
    others from running.

6. Known Limitations
    - The drone_mapper_simulation executable currently runs one hardcoded example scenario
      (data_maps/single_voxel_x2_y4_z2.npy) regardless of the composition-file argument; wiring
      that argument up to actually parse and expand a YAML composition file is not yet done.
    - The mapping algorithm only ever targets the 6 axis-aligned neighbors of a cell (never a
      diagonal), and a LiDAR reading closer than the sensor's configured z_min is reported as
      "PotentiallyOccupied" rather than a precise hit, matching a real close-range sensor
      limitation. A voxel that is only reachable by embedding the drone in solid space (e.g. a
      wall directly at the drone's spawn point, shadowed on all sides) may occasionally remain
      unresolved more precisely than "PotentiallyOccupied".
    - Integration tests (tests/integration/test_integration_real_algorithm.cpp) load their maps
      from a data_maps/ directory relative to the working directory the test binary is run from
      (typically the build/ directory). Those .npy fixtures are provided separately for this
      exercise and are not committed to source control; make sure data_maps/ exists next to
      wherever you invoke drone_mapper_simulation_test from.

7. External Libraries
    mp-units:    Used for implementing strong types to ensure physical unit safety (distances in
                 cm, angles in deg) throughout all internal APIs and calculations.
    TinyNPY:     Lightweight binary parser/writer for the .npy voxel grid map files.
    yaml-cpp:    Used by the maps_comparison utility to optionally parse a comparison
                 configuration file.
    GTest / GMock: Used for the component and integration test suites.

8. Build Instructions
    The project uses CMake, Ninja and vcpkg. Requires the VCPKG_ROOT environment variable to
    point at a vcpkg installation. From the repository root:
        Configure:  cmake --preset default
        Build:      cmake --build build
    The executables (drone_mapper_simulation, maps_comparison, drone_mapper_simulation_test)
    are created directly under build/.

9. Execution
    Run a simulation, optionally providing a composition-file path and an output directory
    (see section 6 for the composition-file caveat):
        ./build/drone_mapper_simulation [composition_file] [output_dir]

    Compare two .npy maps directly, optionally with a YAML comparison config:
        ./build/maps_comparison <origin_map.npy> <target_map.npy> [comparison_config=<path>]

    Run the full test suite (component + integration tests; see section 6 for the data_maps/
    requirement of the integration tests):
        ./build/drone_mapper_simulation_test

10. Bonus Features
    See bonus.txt for the implemented bonus feature (comparing maps with different resolutions
    in MapsComparison) and how to run its dedicated test.
