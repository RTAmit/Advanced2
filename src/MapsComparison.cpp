#include <drone_mapper/MapsComparison.h>

namespace drone_mapper {

std::vector<double> MapsComparison::compare(const IMap3D& origin,
                                            const std::vector<IMap3D*> targets) {
    std::vector<double> results;
    
    types::MapConfig origin_config = origin.getMapConfig();
    int step = origin_config.map_res_cm;
    if (step <= 0) {
        step = 10; 
    }

    for (auto* target : targets) {
        types::MapConfig target_config = target->getMapConfig();
        
        int min_x = target_config.map_boundaries.x_boundary.min_cm;
        int max_x = target_config.map_boundaries.x_boundary.max_cm;
        int min_y = target_config.map_boundaries.y_boundary.min_cm;
        int max_y = target_config.map_boundaries.y_boundary.max_cm;
        int min_z = target_config.map_boundaries.height_boundary.min_cm;
        int max_z = target_config.map_boundaries.height_boundary.max_cm;
        
        long long total_voxels = 0;
        long long matching_voxels = 0;

        for (int x = min_x; x <= max_x; x += step) {
            for (int y = min_y; y <= max_y; y += step) {
                for (int z = min_z; z <= max_z; z += step) {
                    Position3D pos{x, y, z};
                    
    
                    types::VoxelOccupancy v_origin = origin.atVoxel(pos);
                    types::VoxelOccupancy v_target = target->atVoxel(pos);
                    
                    if (v_origin == v_target) {
                        matching_voxels++;
                    }
                    total_voxels++;
                }
            }
        }
        
        double score = 100.0;
        if (total_voxels > 0) {
            score = (static_cast<double>(matching_voxels) / static_cast<double>(total_voxels)) * 100.0;
        }
        results.push_back(score);
    }
    
    return results;
}

} // namespace drone_mapper