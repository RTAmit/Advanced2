#include "drone_mapper/MappingAlgorithmImpl.h"
#include <cmath>

namespace drone_mapper {

MappingAlgorithmImpl::MappingAlgorithmImpl(const types::DroneConfigData drone_config, const IMap3D& output_map)
    : IMappingAlgorithm(drone_config, output_map), is_finished_(false) {}

types::MappingStepCommand MappingAlgorithmImpl::nextStep(const types::DroneState& state, 
                                                         const types::LidarScanResult* latest_scan) {
    (void)latest_scan; // הנחה: מחלקה אחרת (DroneControl או ScanResultToVoxels) כבר מעדכנת את output_map_

    types::MappingStepCommand cmd;
    
    // אם כבר סיימנו לסרוק, נעמוד במקום ונודיע למערכת
    if (is_finished_ || state.battery_level < 5.0) {
        cmd.movement = types::MovementCommand{
            types::MovementCommandType::Rotate, // שימוש ב-Rotate עם 0 מעלות שקול לעמידה במקום
            types::RotationDirection::Left, 
            0.0 * isq::angle::degree, 
            0.0 * si::metre
        };
        // הערה: תלוי במבנה ה-API שלך, אם יש שדה סטטוס ב-MappingStepCommand, עדכן אותו כאן ל-Finished.
        return cmd;
    }

    // אם התור ריק, זה הזמן לייצר נקודות חדשות סביבנו (למשל סריקה עתידית של 4 הכיוונים)
    if (target_queue_.empty()) {
        auto res = output_map_.getMapConfig().resolution;
        
        // מייצרים נקודות פוטנציאליות סביבנו במרחק של רזולוציה אחת
        Position3D p1 = {state.position.x + res, state.position.y, state.position.z};
        Position3D p2 = {state.position.x - res, state.position.y, state.position.z};
        Position3D p3 = {state.position.x, state.position.y + res, state.position.z};
        Position3D p4 = {state.position.x, state.position.y - res, state.position.z};

        // בודקים גבולות וקירות - מכניסים לתור רק אם חוקי ולא נסרק
        for (const auto& p : {p1, p2, p3, p4}) {
            if (output_map_.isInBounds(p) && output_map_.atVoxel(p) == types::VoxelOccupancy::Unmapped) {
                target_queue_.push(p);
            }
        }

        // אם גם אחרי הוספת השכנים התור ריק, סיימנו את המיפוי לחלוטין!
        if (target_queue_.empty()) {
            is_finished_ = true;
            cmd.movement = types::MovementCommand{types::MovementCommandType::Rotate, types::RotationDirection::Left, 0.0 * isq::angle::degree, 0.0 * si::metre};
            return cmd;
        }
    }

    // ניווט לנקודה הבאה בתור
    Position3D next_target = target_queue_.front();
    target_queue_.pop();

    // נוודא שוב רגע לפני הטיסה שהנקודה עדיין פנויה (אולי הליידאר גילה שם קיר בצעד הקודם)
    if (output_map_.atVoxel(next_target) == types::VoxelOccupancy::Occupied) {
        // הנקודה הפכה לקיר, נוותר עליה ונעמוד במקום בצעד הזה
        cmd.movement = types::MovementCommand{types::MovementCommandType::Rotate, types::RotationDirection::Left, 0.0 * isq::angle::degree, 0.0 * si::metre};
        return cmd;
    }

    // חישוב מרחק לנקודה ומתן פקודת התקדמות
    double dx = (next_target.x - state.position.x).force_numerical_value_in(si::metre);
    double dy = (next_target.y - state.position.y).force_numerical_value_in(si::metre);
    double distance_to_target = std::sqrt(dx*dx + dy*dy);

    cmd.movement = types::MovementCommand{
        types::MovementCommandType::Advance, 
        types::RotationDirection::Left, 
        0.0 * isq::angle::degree, 
        distance_to_target * si::metre
    };

    return cmd;
}

} // namespace drone_mapper