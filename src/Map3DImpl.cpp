#include "Map3DImpl.h" 
#include <fstream>
#include <stdexcept>
#include <utility>

namespace drone_mapper {

Map3DImpl::Map3DImpl(std::shared_ptr<NpyArray> map_ptr)
    : Map3DImpl(std::move(map_ptr), types::MapConfig{}) {}

Map3DImpl::Map3DImpl(std::shared_ptr<NpyArray> map_ptr, const types::MapConfig map_config)
    : map_(std::move(map_ptr)),
      config_(map_config) {
    if (!map_) {
        throw std::invalid_argument("Map3DImpl requires a valid map pointer.");
    }
}

bool Map3DImpl::getIndices(const Position3D& pos, int& x_idx, int& y_idx, int& z_idx) const {
    int rel_x = pos.x_cm - config_.map_offset.x_offset;
    int rel_y = pos.y_cm - config_.map_offset.y_offset;
    int rel_z = pos.height_cm - config_.map_offset.height_offset;

    x_idx = rel_x / config_.map_res_cm;
    y_idx = rel_y / config_.map_res_cm;
    z_idx = rel_z / config_.map_res_cm;

    if (x_idx < 0 || x_idx >= static_cast<int>(map_->shape[0]) ||
        y_idx < 0 || y_idx >= static_cast<int>(map_->shape[1]) ||
        z_idx < 0 || z_idx >= static_cast<int>(map_->shape[2])) {
        return false;
    }
    return true;
}

types::VoxelOccupancy Map3DImpl::atVoxel(const Position3D& pos) const {
    int x_idx, y_idx, z_idx;
    if (!getIndices(pos, x_idx, y_idx, z_idx)) {
        return types::VoxelOccupancy::Occupied; 
    }

    size_t flat_idx = x_idx + map_->shape[0] * (y_idx + map_->shape[1] * z_idx);

    uint8_t voxel_val = map_->data[flat_idx];

    if (voxel_val > 0) {
        return types::VoxelOccupancy::Occupied;
    }
    return types::VoxelOccupancy::Free;
}

types::MapConfig Map3DImpl::getMapConfig() const {
    return config_;
}

void Map3DImpl::set(const Position3D& pos, types::VoxelOccupancy value) {
    int x_idx, y_idx, z_idx;
    if (getIndices(pos, x_idx, y_idx, z_idx)) {
        size_t flat_idx = x_idx + map_->shape[0] * (y_idx + map_->shape[1] * z_idx);
        
        if (value == types::VoxelOccupancy::Occupied) {
            map_->data[flat_idx] = 1;
        } else if (value == types::VoxelOccupancy::Free) {
            map_->data[flat_idx] = 0;
        }
    }
}

void Map3DImpl::save(const std::filesystem::path& path) const {
    std::ofstream file(path, std::ios::binary);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open file for writing: " + path.string());
    }

    file << "\x93NUMPY";
    file << (char)0x01 << (char)0x00; 
    
    std::string dict = "{'descr': '|u1', 'fortran_order': False, 'shape': (" + 
                       std::to_string(map_->shape[0]) + ", " + 
                       std::to_string(map_->shape[1]) + ", " + 
                       std::to_string(map_->shape[2]) + "), }";
                       
    int padLen = 64 - ((10 + dict.length() + 1) % 64);
    dict.append(padLen, ' ');
    dict.push_back('\n');

    uint16_t headerLen = dict.length();
    file.write(reinterpret_cast<const char*>(&headerLen), 2);
    file.write(dict.c_str(), dict.length());

    size_t total_size = map_->shape[0] * map_->shape[1] * map_->shape[2];
    file.write(reinterpret_cast<const char*>(map_->data.data()), total_size);
}

} // namespace drone_mapper