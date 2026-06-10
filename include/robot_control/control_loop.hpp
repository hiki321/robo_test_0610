#ifndef ROBOT_CONTROL__CONTROL_LOOP_HPP
#define ROBOT_CONTROL__CONTROL_LOOP_HPP

#include <atomic>
#include <memory>
#include <mutex>
#include <thread>

#include "robot_control/robot_interface.hpp"

namespace robot_control {

class ControlLoop {
public:
    explicit ControlLoop(std::shared_ptr<IRobot> robot, double rate_hz = 50.0);
    ~ControlLoop();

    ControlLoop(const ControlLoop&) = delete;
    ControlLoop& operator=(const ControlLoop&) = delete;

    void start();
    void stop();
    bool running() const;

    void set_velocity(const Twist2D& cmd);
    RobotState state() const;

private:
    void loop();

    std::shared_ptr<IRobot> robot_;
    double rate_hz_;
    std::atomic<bool> running_{false};
    mutable std::mutex command_mutex_;
    Twist2D command_{};
    std::thread worker_;
};

}  // namespace robot_control

#endif  // ROBOT_CONTROL__CONTROL_LOOP_HPP
