#include "MapsComparison.h"
#include <iostream>

namespace drone_mapper {

std::vector<double> MapsComparison::compare(const IMap3D& origin, 
                                            const std::vector<IMap3D*> targets) {
    std::vector<double> results;
    
    types::MapConfig origin_config = origin.getMapConfig();
    // שימוש נכון ביחידות הפיזיקליות החדשות
    int step = origin_config.resolution.force_numerical_value_in(cm);
    if (step <= 0) {
        step = 10; 
    }

    for (auto* target : targets) {
        types::MapConfig target_config = target->getMapConfig();
        
        int min_x = target_config.boundaries.min_x.force_numerical_value_in(cm);
        int max_x = target_config.boundaries.max_x.force_numerical_value_in(cm);
        int min_y = target_config.boundaries.min_y.force_numerical_value_in(cm);
        int max_y = target_config.boundaries.max_y.force_numerical_value_in(cm);
        int min_z = target_config.boundaries.min_height.force_numerical_value_in(cm);
        int max_z = target_config.boundaries.max_height.force_numerical_value_in(cm);
        
        long long total_voxels = 0;
        long long matching_voxels = 0;

        for (int x = min_x; x <= max_x; x += step) {
            for (int y = min_y; y <= max_y; y += step) {
                for (int z = min_z; z <= max_z; z += step) {
                    Position3D pos{XLength{static_cast<double>(x) * cm}, 
                                   YLength{static_cast<double>(y) * cm}, 
                                   ZLength{static_cast<double>(z) * cm}};
                    
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