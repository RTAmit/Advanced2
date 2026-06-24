#include <drone_mapper/Map3DImpl.h>
#include <cmath>

namespace drone_mapper {

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
    unsigned long ix = static_cast<unsigned long>(std::floor((pos.x - m_config.boundaries.min_x) / m_config.resolution));
    unsigned long iy = static_cast<unsigned long>(std::floor((pos.y - m_config.boundaries.min_y) / m_config.resolution));
    unsigned long iz = static_cast<unsigned long>(std::floor((pos.z - m_config.boundaries.min_height) / m_config.resolution));
    return {ix, iy, iz};
}

types::VoxelOccupancy Map3DImpl::atVoxel(const Position3D& pos) const {
    if (!isInBounds(pos)) {
        return types::VoxelOccupancy::OutOfBounds; 
    }

    std::vector<unsigned long> indices = positionToIndices(pos);
    
    // 1. שימוש בפונקציה GetShape במקום גישה ישירה למשתנה הפרטי
    const auto& shape = m_npy_array->GetShape(); 

    if (indices[0] >= shape[0] || 
        indices[1] >= shape[1] || 
        indices[2] >= shape[2]) {
        return types::VoxelOccupancy::OutOfBounds; 
    }

    // 2. חישוב האינדקס השטוח (Flat Index) במערך הזיכרון
    size_t flat_idx = indices[0] * shape[1] * shape[2] + indices[1] * shape[2] + indices[2];
    
    // 3. שליפת הערך בעזרת הפונקציה Data של הספרייה החדשה
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
    
    // 1. שימוש ב-GetShape
    const auto& shape = m_npy_array->GetShape(); 

    if (indices[0] >= shape[0] || 
        indices[1] >= shape[1] || 
        indices[2] >= shape[2]) {
        return;
    }

    uint8_t val = 255; 
    switch (state) {
        case types::VoxelOccupancy::Empty: 
            val = 0; 
            break;
        case types::VoxelOccupancy::Occupied: 
            val = 1; 
            break;
        case types::VoxelOccupancy::PotentiallyOccupied: 
            val = 2; 
            break;
        case types::VoxelOccupancy::Unmapped: 
            val = 255;
            break;
        case types::VoxelOccupancy::OutOfBounds: 
            return; 
    }

    // 2. חישוב האינדקס השטוח וכתיבה ישירה לזיכרון דרך Data()
    size_t flat_idx = indices[0] * shape[1] * shape[2] + indices[1] * shape[2] + indices[2];
    m_npy_array->Data<uint8_t>()[flat_idx] = val;
}

} // namespace drone_mapper