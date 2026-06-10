#include "robot_control/differential_drive_sim.hpp"

#include <algorithm>
#include <cmath>

namespace robot_control {

namespace {

double clamp(double value, double min_value, double max_value) {
    return std::max(min_value, std::min(value, max_value));
}

}  // namespace

DifferentialDriveSim::DifferentialDriveSim(DifferentialDriveConfig config)
    : config_(config) {}

void DifferentialDriveSim::set_velocity(const Twist2D& cmd) {
    command_ = cmd;
}

RobotState DifferentialDriveSim::state() const {
    return state_;
}

void DifferentialDriveSim::update(double dt) {
    state_.wheels = twist_to_wheels(command_, config_);
    state_.velocity = wheels_to_twist(state_.wheels, config_);

    const double v = state_.velocity.linear_x;
    const double w = state_.velocity.angular_z;

    if (std::abs(w) < 1e-9) {
        state_.pose.x += v * std::cos(state_.pose.theta) * dt;
        state_.pose.y += v * std::sin(state_.pose.theta) * dt;
    } else {
        const double radius = v / w;
        state_.pose.x += radius * (std::sin(state_.pose.theta + w * dt) - std::sin(state_.pose.theta));
        state_.pose.y += radius * (-std::cos(state_.pose.theta + w * dt) + std::cos(state_.pose.theta));
        state_.pose.theta += w * dt;
    }

    state_.pose.normalize_theta();
}

void DifferentialDriveSim::reset(const Pose2D& pose) {
    command_ = {};
    state_ = {};
    state_.pose = pose;
}

WheelVelocities DifferentialDriveSim::twist_to_wheels(const Twist2D& twist,
                                                      const DifferentialDriveConfig& config) {
    const double half_track = config.wheel_separation * 0.5;
    WheelVelocities wheels;
    wheels.left = (twist.linear_x - twist.angular_z * half_track) / config.wheel_radius;
    wheels.right = (twist.linear_x + twist.angular_z * half_track) / config.wheel_radius;

    const double max_speed = config.max_wheel_speed;
    wheels.left = clamp(wheels.left, -max_speed, max_speed);
    wheels.right = clamp(wheels.right, -max_speed, max_speed);
    return wheels;
}

Twist2D DifferentialDriveSim::wheels_to_twist(const WheelVelocities& wheels,
                                               const DifferentialDriveConfig& config) {
    const double half_track = config.wheel_separation * 0.5;
    Twist2D twist;
    twist.linear_x = config.wheel_radius * (wheels.left + wheels.right) * 0.5;
    twist.angular_z = config.wheel_radius * (wheels.right - wheels.left) / config.wheel_separation;
    (void)half_track;
    return twist;
}

}  // namespace robot_control
