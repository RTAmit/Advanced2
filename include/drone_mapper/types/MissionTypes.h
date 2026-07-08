#pragma once

#include <drone_mapper/Units.h>
#include <drone_mapper/types/MapTypes.h>

#include <cstddef>
#include <string>
#include <vector>

namespace drone_mapper::types {

struct MissionConfigData {
    std::size_t max_steps = 0;
    PhysicalLength gps_resolution{};
    double output_mapping_resolution_factor = 0;
    // The mission's operational area, as read from mission_config.yaml. This
    // constrains where the drone is allowed to fly and is applied to the
    // output map's boundaries; it may be a smaller region than the full
    // extent of the underlying map array. Placed last so existing positional
    // aggregate-initializers (e.g. MissionConfigData{2000, 10.0*cm, 1}) still
    // compile with a default-constructed (empty) bounds value.
    MappingBounds mission_bounds{};
};

enum class MissionRunStatus {
    Completed,
    MaxSteps,
    Error,
};

struct ErrorRef {
    std::string code{};
    std::string message{};
};

struct MissionRunResult {
    MissionRunStatus status = MissionRunStatus::Completed;
    std::size_t steps = 0;
    // Changed: a run can report multiple errors instead of a single ErrorRef.
    std::vector<ErrorRef> errors{}; // we may have multiple errors

    //Removed in 9.6
    // double score = 0.0; // moved to simulationResult
    // std::filesystem::path output_map_file{}; // moved to simulation Result
};

} // namespace drone_mapper::types