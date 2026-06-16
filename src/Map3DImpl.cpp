#include "Map3DImpl.h"
#include <string>
#include <utility>

namespace drone_mapper {

// מימוש הבנאי הראשון
Map3DImpl::Map3DImpl(std::shared_ptr<NpyArray> map_ptr)
    : map_(std::move(map_ptr)) {
}

// מימוש הבנאי השני (שמקבל גם קונפיגורציה)
Map3DImpl::Map3DImpl(std::shared_ptr<NpyArray> map_ptr, const types::MapConfig map_config)
    : map_(std::move(map_ptr)), config_(map_config) {
}

bool Map3DImpl::getIndices(const Position3D& pos, int& x_idx, int& y_idx, int& z_idx) const {
    int rel_x = pos.x.force_numerical_value_in(cm) - config_.offset.x.force_numerical_value_in(cm);
    int rel_y = pos.y.force_numerical_value_in(cm) - config_.offset.y.force_numerical_value_in(cm);
    int rel_z = pos.z.force_numerical_value_in(cm) - config_.offset.z.force_numerical_value_in(cm);

    int res = config_.resolution.force_numerical_value_in(cm);
    if (res <= 0) res = 10;

    x_idx = rel_x / res;
    y_idx = rel_y / res;
    z_idx = rel_z / res;

    if (x_idx < 0 || x_idx >= static_cast<int>(map_->Shape()[0]) ||
        y_idx < 0 || y_idx >= static_cast<int>(map_->Shape()[1]) ||
        z_idx < 0 || z_idx >= static_cast<int>(map_->Shape()[2])) {
        return false; 
    }
    return true;
}

types::VoxelOccupancy Map3DImpl::atVoxel(const Position3D& pos) const {
    int x_idx, y_idx, z_idx;
    if (!getIndices(pos, x_idx, y_idx, z_idx)) {
        return types::VoxelOccupancy::Occupied; 
    }
    size_t flat_idx = x_idx + map_->Shape()[0] * (y_idx + map_->Shape()[1] * z_idx);
    uint8_t voxel_val = map_->Data()[flat_idx];

    if (voxel_val > 0) {
        return types::VoxelOccupancy::Occupied;
    }
    return types::VoxelOccupancy::Empty; 
}

void Map3DImpl::set(const Position3D& pos, types::VoxelOccupancy value) {
    int x_idx, y_idx, z_idx;
    if (!getIndices(pos, x_idx, y_idx, z_idx)) return;
    
    size_t flat_idx = x_idx + map_->Shape()[0] * (y_idx + map_->Shape()[1] * z_idx);
    uint8_t* mut_data = const_cast<uint8_t*>(map_->Data());
    
    if (value == types::VoxelOccupancy::Occupied) {
        mut_data[flat_idx] = 1;
    } else if (value == types::VoxelOccupancy::Empty) {
        mut_data[flat_idx] = 0;
    }
}

void Map3DImpl::save(const std::filesystem::path& path) const {
    (void)path; // Stub save function
}

types::MapConfig Map3DImpl::getMapConfig() const {
    return config_;
}

} // namespace drone_mapper