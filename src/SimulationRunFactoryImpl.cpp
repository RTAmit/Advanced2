#include "drone_mapper/SimulationRunFactoryImpl.h"
#include "drone_mapper/DroneControlImpl.h"
#include "drone_mapper/ErrorLog.h"
#include "drone_mapper/Map3DImpl.h"
#include "drone_mapper/MappingAlgorithmImpl.h"
#include "drone_mapper/MissionControlImpl.h"
#include "drone_mapper/MockGPS.h"
#include "drone_mapper/MockLidar.h"
#include "drone_mapper/MockMovement.h"
#include "drone_mapper/SimulationRunImpl.h"
#include "drone_mapper/Units.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <vector>

using namespace mp_units::si::unit_symbols;

namespace drone_mapper {

namespace {

// True for a default-constructed (all-zero) MappingBounds, used as the
// sentinel meaning "mission_config.yaml did not specify boundaries" -> fall
// back to the full extent of the underlying map array.
bool isUnsetBounds(const types::MappingBounds& b) {
    return b.min_x == b.max_x && b.min_y == b.max_y && b.min_height == b.max_height &&
           b.min_x.force_numerical_value_in(cm) == 0.0 && b.max_x.force_numerical_value_in(cm) == 0.0;
}

// World-space extent covered by an array of the given shape, given the
// world<->array offset (array_position = world_position + offset).
types::MappingBounds fullExtentBounds(const NpyArray::shape_t& shape, double resolution_cm,
                                      const Position3D& offset) {
    const double ox = offset.x.force_numerical_value_in(cm);
    const double oy = offset.y.force_numerical_value_in(cm);
    const double oz = offset.z.force_numerical_value_in(cm);
    return types::MappingBounds{
        (0.0 - ox) * cm, (static_cast<double>(shape[0]) * resolution_cm - ox) * cm,
        (0.0 - oy) * cm, (static_cast<double>(shape[1]) * resolution_cm - oy) * cm,
        (0.0 - oz) * cm, (static_cast<double>(shape[2]) * resolution_cm - oz) * cm,
    };
}

// Validates output_mapping_resolution_factor (spec: optional integer, >=1;
// defaults to 1 if missing, ignored with an immediate error log if < 1) and
// returns the resulting output map resolution alongside the status to
// report back in the run's result.
std::pair<double, types::ResolutionRequestStatus> resolveOutputResolutionCm(
    const types::MissionConfigData& mission, double fallback_resolution_cm) {
    const double gps_resolution_cm = mission.gps_resolution.force_numerical_value_in(cm) > 0.0
                                          ? mission.gps_resolution.force_numerical_value_in(cm)
                                          : fallback_resolution_cm;

    if (mission.output_mapping_resolution_factor == 0.0) {
        // Not specified in the config -> defaults to factor 1.
        return {gps_resolution_cm, types::ResolutionRequestStatus::Accepted};
    }
    if (mission.output_mapping_resolution_factor < 1.0) {
        ErrorLog::log("RESOLUTION_FACTOR_TOO_SMALL",
                     "output_mapping_resolution_factor must be an integer >= 1; ignoring the "
                     "request and using factor 1 instead");
        return {gps_resolution_cm, types::ResolutionRequestStatus::IgnoredTooSmall};
    }
    const double factor = std::round(mission.output_mapping_resolution_factor);
    return {gps_resolution_cm * factor, types::ResolutionRequestStatus::Accepted};
}

} // namespace

std::unique_ptr<ISimulationRun>
SimulationRunFactoryImpl::create(const types::SimulationConfigData& simulation,
                                 const types::MissionConfigData& mission,
                                 const types::DroneConfigData& drone,
                                 const types::LidarConfigData& lidar,
                                 const std::filesystem::path& output_path) {

    auto hidden_npy = std::make_shared<NpyArray>();
    hidden_npy->LoadNPY(simulation.map_filename.string());

    const double input_resolution_cm = simulation.map_resolution.force_numerical_value_in(cm) > 0.0
                                            ? simulation.map_resolution.force_numerical_value_in(cm)
                                            : 10.0;

    types::MapConfig hidden_config;
    hidden_config.resolution = input_resolution_cm * cm;
    hidden_config.offset = simulation.map_offset;
    hidden_config.boundaries =
        fullExtentBounds(hidden_npy->Shape(), input_resolution_cm, simulation.map_offset);

    auto hidden_map = std::make_unique<Map3DImpl>(hidden_npy, hidden_config);

    // The mission's operational area: the configured boundaries if present,
    // otherwise the whole extent of the input map.
    const types::MappingBounds mission_bounds = isUnsetBounds(mission.mission_bounds)
                                                     ? hidden_config.boundaries
                                                     : mission.mission_bounds;

    const auto [output_resolution_cm, resolution_status] =
        resolveOutputResolutionCm(mission, input_resolution_cm);

    const double width_cm =
        mission_bounds.max_x.force_numerical_value_in(cm) - mission_bounds.min_x.force_numerical_value_in(cm);
    const double depth_cm =
        mission_bounds.max_y.force_numerical_value_in(cm) - mission_bounds.min_y.force_numerical_value_in(cm);
    const double height_cm = mission_bounds.max_height.force_numerical_value_in(cm) -
                             mission_bounds.min_height.force_numerical_value_in(cm);

    NpyArray::shape_t output_shape = {
        static_cast<size_t>(std::max(1.0, std::ceil(width_cm / output_resolution_cm))),
        static_cast<size_t>(std::max(1.0, std::ceil(depth_cm / output_resolution_cm))),
        static_cast<size_t>(std::max(1.0, std::ceil(height_cm / output_resolution_cm))),
    };

    auto output_npy = std::make_shared<NpyArray>(output_shape, 1, 'u', false);
    output_npy->Allocate();
    std::fill_n(output_npy->Data<uint8_t>(),
               output_shape[0] * output_shape[1] * output_shape[2],
               255); // Unmapped

    types::MapConfig output_config;
    output_config.resolution = output_resolution_cm * cm;
    output_config.boundaries = mission_bounds;
    // The output map is its own independent array: its origin is the corner
    // of its own operational area, not the hidden map's origin.
    output_config.offset =
        Position3D{-mission_bounds.min_x, -mission_bounds.min_y, -mission_bounds.min_height};

    auto output_map = std::make_unique<Map3DImpl>(output_npy, output_config);

    auto gps = std::make_unique<MockGPS>(
        simulation.initial_drone_position,
        Orientation{simulation.initial_angle, 0.0 * altitude_angle[deg]},
        mission.gps_resolution);

    auto movement = std::make_unique<MockMovement>(*gps);
    auto lidar_impl = std::make_unique<MockLidar>(lidar, *hidden_map, *gps);
    auto mapping_algorithm = std::make_unique<MappingAlgorithmImpl>(mission, lidar, drone, *output_map);

    auto drone_control = std::make_unique<DroneControlImpl>(
        drone, mission, *lidar_impl, *gps, *movement, *output_map, *mapping_algorithm);

    const std::filesystem::path output_map_file = output_path / "map_output.npy";

    auto mission_control = std::make_unique<MissionControlImpl>(
        mission, drone, *hidden_map, *output_map, *drone_control, output_map_file);

    return std::make_unique<SimulationRunImpl>(
        std::move(hidden_map),
        std::move(output_map),
        std::move(gps),
        std::move(movement),
        std::move(lidar_impl),
        std::move(mapping_algorithm),
        std::move(drone_control),
        std::move(mission_control),
        simulation,
        mission,
        output_map_file,
        resolution_status
    );
}

} // namespace drone_mapper
