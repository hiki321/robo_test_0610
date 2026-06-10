#include <cassert>
#include <cmath>

#include "robot_control/differential_drive_sim.hpp"

int main() {
    using namespace robot_control;

    DifferentialDriveConfig config;
    config.max_wheel_speed = 20.0;
    DifferentialDriveSim robot(config);
    robot.set_velocity({0.5, 0.0});

    for (int i = 0; i < 50; ++i) {
        robot.update(0.02);
    }

    const auto state = robot.state();
    assert(state.pose.x > 0.4);
    assert(std::abs(state.pose.y) < 1e-6);
    assert(std::abs(state.velocity.angular_z) < 1e-6);

    robot.reset();
    robot.set_velocity({0.0, 1.0});
    for (int i = 0; i < 25; ++i) {
        robot.update(0.02);
    }

    const auto turn_state = robot.state();
    assert(std::abs(turn_state.pose.theta - 0.5) < 0.05);
    return 0;
}
