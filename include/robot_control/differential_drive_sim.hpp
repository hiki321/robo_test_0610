#ifndef ROBOT_CONTROL__DIFFERENTIAL_DRIVE_SIM_HPP
#define ROBOT_CONTROL__DIFFERENTIAL_DRIVE_SIM_HPP

#include "robot_control/robot_interface.hpp"

namespace robot_control {

struct DifferentialDriveConfig {
    double wheel_radius{0.05};
    double wheel_separation{0.3};
    double max_wheel_speed{5.0};
};

class DifferentialDriveSim final : public IRobot {
public:
    explicit DifferentialDriveSim(DifferentialDriveConfig config = {});

    void set_velocity(const Twist2D& cmd) override;
    RobotState state() const override;
    void update(double dt) override;
    void reset(const Pose2D& pose = {}) override;

    static WheelVelocities twist_to_wheels(const Twist2D& twist,
                                           const DifferentialDriveConfig& config);
    static Twist2D wheels_to_twist(const WheelVelocities& wheels,
                                   const DifferentialDriveConfig& config);

private:
    DifferentialDriveConfig config_;
    Twist2D command_;
    RobotState state_;
};

}  // namespace robot_control

#endif  // ROBOT_CONTROL__DIFFERENTIAL_DRIVE_SIM_HPP
