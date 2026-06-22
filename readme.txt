Project Name: Drone Mapper Simulation - Assignment 2
Course: Advanced Topics in Programming (2026B)
Contributors: Amit Reif-Tagari 322698986

1. General Description
    This project simulates an autonomous drone designed for 3D mapping under multiple scenario configurations.
    The system executes a batch of simulation runs by parsing a central composition file and computing a 
    Cartesian product of combinations, validating drone capabilities, lidar specifications, and mission boundaries.

2. Input File Formats
    The program processes YAML configuration files and binary world maps:  
        simulation_compositions.yaml: A central composition file that defines the combinations and Cartesian 
                                      product of all simulation runs to be executed.
        Configuration Files (.yaml):  Individual configuration files defining specific drone capabilities, 
                                      mission boundaries, and lidar specifications.  
        Map Files (.npy):             Binary files containing a 3D voxel grid. The simulation uses an internal 
                                      lightweight parser to extract the binary 3D structure.

3. Output File Format and Directory Structure
    The simulation generates structural hierarchical reports, maps, and structured logs:  
        map_output.txt:          The generated map output saved as a binary .npy file (named map_output.txt 
                                 per assignment instructions despite the extension requested in some places, 
                                 written in binary to match input).
        simulation_output.yaml:  A structured hierarchical YAML report detailing the score_report, summary 
                                 of runs, and specific details for each scenario combination including errors (-1 score).
        output_results/:         A dedicated output directory under which all individual run logs and maps 
                                 are organized chronologically or by scenario name to prevent overriding.

4. Error Handling and Fault Isolation
    The system is designed to be resilient and handle simulation errors or invalid scenario combinations gracefully.
    Scenario combinations that encounter structural initialization failures, invalid parameters, or runtime 
    exceptions are safely caught and documented in the final report with an error status indication (-1 score), 
    ensuring that the overarching batch execution continues seamlessly without crashing.

5. External Libraries
    mp-units: Used for implementing Strong Types to ensure physical unit safety (distances in cm, angles in deg). 
              As per the assignment instructions, this library is used for all internal APIs and calculations 
              to ensure mathematical correctness.
    yaml-cpp: Used for parsing configuration files and generating the structured hierarchical output reports.
    GTest / GMock: Used for the components and integration testing frameworks to guarantee system correctness.

6. Build Instructions
    The project uses CMake and vcpkg. To build the project on Linux:  
        Create a build directory: mkdir build && cd build
        Configure with presets:   cmake --preset default
        Build the target:        cmake --build build
    The executables will be created in the build folder.

7. Execution
    Run the simulation program by providing the path to the central configuration file:
    ./build/drone_mapper_simulation <path_to_simulation_compositions.yaml>
    
    To run the comprehensive test suite:
    ./build/drone_mapper_simulation_test