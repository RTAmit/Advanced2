#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/Units.h>
#include <algorithm>
#include <cmath>

using namespace mp_units::si::unit_symbols;

namespace drone_mapper {

Map3DImpl::Map3DImpl(std::shared_ptr<NpyArray> map_ptr)
    : m_npy_array(std::move(map_ptr)) {}

Map3DImpl::Map3DImpl(std::shared_ptr<NpyArray> npy_array, const types::MapConfig& config)
    : m_npy_array(std::move(npy_array)), m_config(config) {}

types::MapConfig Map3DImpl::getMapConfig() const {
    return m_config;
}

bool Map3DImpl::isInBounds(const Position3D& pos) const {
    const auto& b = m_config.boundaries;
    // Positions reached via trigonometric movement (sin/cos of a heading
    // angle) can land a floating-point epsilon outside an exact boundary
    // even when the drone is mathematically moving to an in-bounds voxel
    // (e.g. cos(90 deg) is ~6e-17, not exactly 0). Tolerate that noise
    // instead of rejecting a legitimately in-bounds position.
    constexpr double kEpsilonCm = 1e-6;
    const double x = pos.x.force_numerical_value_in(cm);
    const double y = pos.y.force_numerical_value_in(cm);
    const double z = pos.z.force_numerical_value_in(cm);
    const double min_x = b.min_x.force_numerical_value_in(cm);
    const double max_x = b.max_x.force_numerical_value_in(cm);
    const double min_y = b.min_y.force_numerical_value_in(cm);
    const double max_y = b.max_y.force_numerical_value_in(cm);
    const double min_z = b.min_height.force_numerical_value_in(cm);
    const double max_z = b.max_height.force_numerical_value_in(cm);

    return x >= min_x - kEpsilonCm && x <= max_x + kEpsilonCm &&
           y >= min_y - kEpsilonCm && y <= max_y + kEpsilonCm &&
           z >= min_z - kEpsilonCm && z <= max_z + kEpsilonCm;
}

std::vector<unsigned long> Map3DImpl::positionToIndices(const Position3D& pos) const {
    double x_ratio = static_cast<double>((pos.x - m_config.boundaries.min_x) / m_config.resolution);
    double y_ratio = static_cast<double>((pos.y - m_config.boundaries.min_y) / m_config.resolution);
    double z_ratio = static_cast<double>((pos.z - m_config.boundaries.min_height) / m_config.resolution);

    // isInBounds() tolerates a tiny epsilon past the exact edge, so the ratio
    // here can come out marginally negative or marginally >= extent for a
    // position that is otherwise legitimately in-bounds. Clamp into the
    // valid voxel-index range rather than let it wrap (negative -> huge
    // unsigned long) or spuriously read back as out-of-bounds.
    const auto& shape = m_npy_array->Shape();
    auto toClampedIndex = [](double ratio, unsigned long extent) -> unsigned long {
        if (extent == 0) {
            return 0;
        }
        if (ratio <= 0.0) {
            return 0;
        }
        const double floored = std::floor(ratio);
        if (floored >= static_cast<double>(extent)) {
            return extent - 1;
        }
        return static_cast<unsigned long>(floored);
    };

    unsigned long ix = toClampedIndex(x_ratio, shape[0]);
    unsigned long iy = toClampedIndex(y_ratio, shape[1]);
    unsigned long iz = toClampedIndex(z_ratio, shape[2]);
    return {ix, iy, iz};
}

types::VoxelOccupancy Map3DImpl::atVoxel(const Position3D& pos) const {
    if (!isInBounds(pos)) {
        return types::VoxelOccupancy::OutOfBounds; 
    }

    std::vector<unsigned long> indices = positionToIndices(pos);
    const auto& shape = m_npy_array->Shape(); 

    if (indices[0] >= shape[0] || 
        indices[1] >= shape[1] || 
        indices[2] >= shape[2]) {
        return types::VoxelOccupancy::OutOfBounds; 
    }

    size_t flat_idx = indices[0] * shape[1] * shape[2] + indices[1] * shape[2] + indices[2];
    uint8_t val = m_npy_array->Data<uint8_t>()[flat_idx];

    switch (val) {
        case 0: return types::VoxelOccupancy::Empty;
        case 1: return types::VoxelOccupancy::Occupied;
        case 2: return types::VoxelOccupancy::PotentiallyOccupied; 
        default: return types::VoxelOccupancy::Unmapped; 
    }
}

void Map3DImpl::set(const Position3D& pos, types::VoxelOccupancy state) {
    if (!isInBounds(pos)) {
        return; 
    }

    std::vector<unsigned long> indices = positionToIndices(pos);
    const auto& shape = m_npy_array->Shape(); 

    if (indices[0] >= shape[0] || 
        indices[1] >= shape[1] || 
        indices[2] >= shape[2]) {
        return;
    }

    uint8_t val = 255; 
    switch (state) {
        case types::VoxelOccupancy::Empty: val = 0; break;
        case types::VoxelOccupancy::Occupied: val = 1; break;
        case types::VoxelOccupancy::PotentiallyOccupied: val = 2; break;
        case types::VoxelOccupancy::Unmapped: val = 255; break;
        case types::VoxelOccupancy::OutOfBounds: return; 
    }

    size_t flat_idx = indices[0] * shape[1] * shape[2] + indices[1] * shape[2] + indices[2];
    m_npy_array->Data<uint8_t>()[flat_idx] = val;
}

void Map3DImpl::save(const std::filesystem::path& output_path) const {
    if (m_npy_array) {
        m_npy_array->SaveNPY(output_path.string());
    }
}

} // namespace drone_mapper