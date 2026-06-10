#ifndef ROBOT_CONTROL__ROBOT_INTERFACE_HPP
#define ROBOT_CONTROL__ROBOT_INTERFACE_HPP

#include "robot_control/types.hpp"

namespace robot_control {

// Hardware-agnostic robot interface. Real hardware drivers implement this later.
class IRobot {
public:
    virtual ~IRobot() = default;

    virtual void set_velocity(const Twist2D& cmd) = 0;
    virtual RobotState state() const = 0;
    virtual void update(double dt) = 0;
    virtual void reset(const Pose2D& pose = {}) = 0;
};

}  // namespace robot_control

#endif  // ROBOT_CONTROL__ROBOT_INTERFACE_HPP
