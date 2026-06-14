#pragma once

#include <drone_mapper/Units.h>

namespace drone_mapper::types {

enum class VoxelOccupancy {
    OutOfBounds = -2,
    Unmapped = -1,
    Empty = 0,
    Occupied = 1,
};

struct MappingBounds {
    XLength min_x{};
    XLength max_x{};
    YLength min_y{};
    YLength max_y{};
    ZLength min_height{};
    ZLength max_height{};
};

struct MappedVoxel {
    Position3D position{};
    VoxelOccupancy value = VoxelOccupancy::Unmapped;
};

struct MapConfig {
    MappingBounds boundaries{};
    Position3D offset{};
    PhysicalLength resolution{};
};
} // namespace drone_mapper::types