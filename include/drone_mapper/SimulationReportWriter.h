#pragma once

#include <drone_mapper/Types.h>

#include <filesystem>

namespace drone_mapper {

// Writes simulation_output.yaml: a hierarchical score report (simulations ->
// missions -> per drone/lidar runs), matching the assignment's example
// layout. `composition` and `report` must come from the same
// SimulationManager::run() call, since this walks both in lockstep (the
// flat report.runs list is produced in the same nested order the
// composition was iterated in) to reconstruct the grouping.
class SimulationReportWriter {
public:
    static void write(const types::SimulationCompositionData& composition,
                      const types::SimulationManagerReport& report,
                      const std::filesystem::path& output_dir);
};

} // namespace drone_mapper
