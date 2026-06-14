#include <drone_mapper/MockMovement.h>
#include <mp-units/systems/si/math.h>

namespace drone_mapper {

MockMovement::MockMovement(MockGPS& gps) : gps_(gps) {}

types::MovementResult MockMovement::rotate(types::RotationDirection direction, HorizontalAngle angle) {
    const Orientation current = gps_.heading();
    const HorizontalAngle signed_angle =
        (direction == types::RotationDirection::Left) ? angle : -angle;
    
    gps_.setHeading(Orientation{current.horizontal + signed_angle, current.altitude});
    return types::MovementResult{true, {}};
}

types::MovementResult MockMovement::advance(PhysicalLength distance) {
    const Orientation current = gps_.heading();
    Position3D pos = gps_.position();
    
    const auto cos_altitude = si::cos(current.altitude);
    const auto dx = cos_altitude * si::cos(current.horizontal);
    const auto dy = cos_altitude * si::sin(current.horizontal);
    const auto dz = si::sin(current.altitude);
    
    pos.x += distance * dx;
    pos.y += distance * dy;
    pos.z += distance * dz;
    
    gps_.setPosition(pos);
    return types::MovementResult{true, {}};
}

types::MovementResult MockMovement::elevate(PhysicalLength distance) {
    Position3D pos = gps_.position();
    
    pos.z += distance;
    
    gps_.setPosition(pos);
    return types::MovementResult{true, {}};
}

} // namespace drone_mapper