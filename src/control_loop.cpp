#include "robot_control/control_loop.hpp"

#include <chrono>
#include <thread>

namespace robot_control {

ControlLoop::ControlLoop(std::shared_ptr<IRobot> robot, double rate_hz)
    : robot_(std::move(robot)), rate_hz_(rate_hz) {}

ControlLoop::~ControlLoop() {
    stop();
}

void ControlLoop::start() {
    if (running_) {
        return;
    }

    running_ = true;
    worker_ = std::thread([this] { loop(); });
}

void ControlLoop::stop() {
    if (!running_) {
        return;
    }

    running_ = false;
    if (worker_.joinable()) {
        worker_.join();
    }
}

bool ControlLoop::running() const {
    return running_;
}

void ControlLoop::set_velocity(const Twist2D& cmd) {
    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        command_ = cmd;
    }
    robot_->set_velocity(cmd);
}

RobotState ControlLoop::state() const {
    return robot_->state();
}

void ControlLoop::loop() {
    using clock = std::chrono::steady_clock;
    const auto period = std::chrono::duration<double>(1.0 / rate_hz_);
    auto next_tick = clock::now();

    while (running_) {
        const auto now = clock::now();
        const double dt = std::chrono::duration<double>(now - (next_tick - period)).count();

        Twist2D cmd;
        {
            std::lock_guard<std::mutex> lock(command_mutex_);
            cmd = command_;
        }
        robot_->set_velocity(cmd);
        robot_->update(dt);

        next_tick += std::chrono::duration_cast<clock::duration>(period);
        std::this_thread::sleep_until(next_tick);
    }
}

}  // namespace robot_control
