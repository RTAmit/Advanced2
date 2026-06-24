#include <drone_mapper/Map3DImpl.h>
#include <cmath>

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
    return pos.x >= b.min_x && pos.x <= b.max_x &&
           pos.y >= b.min_y && pos.y <= b.max_y &&
           pos.z >= b.min_height && pos.z <= b.max_height;
}

std::vector<unsigned long> Map3DImpl::positionToIndices(const Position3D& pos) const {
    double x_ratio = static_cast<double>((pos.x - m_config.boundaries.min_x) / m_config.resolution);
    double y_ratio = static_cast<double>((pos.y - m_config.boundaries.min_y) / m_config.resolution);
    double z_ratio = static_cast<double>((pos.z - m_config.boundaries.min_height) / m_config.resolution);

    unsigned long ix = static_cast<unsigned long>(std::floor(x_ratio));
    unsigned long iy = static_cast<unsigned long>(std::floor(y_ratio));
    unsigned long iz = static_cast<unsigned long>(std::floor(z_ratio));
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