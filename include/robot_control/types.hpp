#ifndef ROBOT_CONTROL__TYPES_HPP
#define ROBOT_CONTROL__TYPES_HPP

#include <cmath>

namespace robot_control {

struct Pose2D {
    double x{0.0};
    double y{0.0};
    double theta{0.0};

    void normalize_theta() {
        while (theta > M_PI) {
            theta -= 2.0 * M_PI;
        }
        while (theta < -M_PI) {
            theta += 2.0 * M_PI;
        }
    }
};

struct Twist2D {
    double linear_x{0.0};
    double angular_z{0.0};
};

struct WheelVelocities {
    double left{0.0};
    double right{0.0};
};

struct RobotState {
    Pose2D pose;
    Twist2D velocity;
    WheelVelocities wheels;
};

}  // namespace robot_control

#endif  // ROBOT_CONTROL__TYPES_HPP
