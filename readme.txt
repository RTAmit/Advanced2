Advanced Topics in Programming - Assignment 2 (2026B)
=====================================================

Contributors:
-------------
Name: Amit Reif-Tagari, ID: 322698986

Input Formats:
--------------
- Map Files: Binary .npy files containing a 3D voxel grid. The simulation uses an internal lightweight parser to extract the binary 3D structure.
- Configuration Files: YAML formats (.yaml) defining drone capabilities, mission boundaries, lidar specifications, and a central composition file (`simulation_compositions.yaml`) that runs a Cartesian product of all the runs.

Output Formats:
---------------
- Map Output: Binary .npy file (named map_output.txt per assignment instructions despite the extension requested in some places, written in binary to match input).
- Simulation Output (simulation_output.yaml): A structured hierarchical YAML report detailing the score_report, summary of runs, and specific details for each scenario combination including errors (-1 score).
- Output Directory: Under the requested output_results folder, all individual run logs and maps are organized chronologically or by scenario name to prevent overriding.

External Libraries:
-------------------
- yaml-cpp: Used for parsing configuration files and generating the structured output report.
- GTest / GMock: Used for the components and integration testing frameworks.