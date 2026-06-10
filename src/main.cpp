#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <iostream>
#include <memory>
#include <termios.h>
#include <thread>
#include <unistd.h>

#include "robot_control/control_loop.hpp"
#include "robot_control/differential_drive_sim.hpp"

namespace {

std::atomic<bool> g_running{true};

void handle_signal(int) {
    g_running = false;
}

termios original_termios{};

void enable_raw_mode() {
    if (tcgetattr(STDIN_FILENO, &original_termios) == -1) {
        return;
    }

    termios raw = original_termios;
    raw.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 1;
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw);
}

void restore_terminal() {
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &original_termios);
}

robot_control::Twist2D key_to_twist(char key) {
    constexpr double linear = 0.5;
    constexpr double angular = 1.0;

    switch (key) {
        case 'w':
        case 'W':
            return {linear, 0.0};
        case 's':
        case 'S':
            return {-linear, 0.0};
        case 'a':
        case 'A':
            return {0.0, angular};
        case 'd':
        case 'D':
            return {0.0, -angular};
        case 'q':
        case 'Q':
            return {linear * 0.5, angular};
        case 'e':
        case 'E':
            return {linear * 0.5, -angular};
        default:
            return {};
    }
}

void print_state(const robot_control::RobotState& state) {
    std::printf(
        "\rpose(x=%6.2f, y=%6.2f, theta=%6.2f)  vel(v=%5.2f, w=%5.2f)  wheels(L=%5.2f, R=%5.2f)   ",
        state.pose.x,
        state.pose.y,
        state.pose.theta,
        state.velocity.linear_x,
        state.velocity.angular_z,
        state.wheels.left,
        state.wheels.right);
    std::fflush(stdout);
}

void print_help() {
    std::cout << "Robot Control (pure C++)\n"
              << "  W/S : forward / backward\n"
              << "  A/D : turn left / right\n"
              << "  Q/E : arc left / right\n"
              << "  Space: stop\n"
              << "  X   : exit\n";
}

}  // namespace

int main() {

    std::cout << "Robot Control (pure C++)\n";
    std::cout << "  W/S : forward / backward\n"
              << "  A/D : turn left / right\n"
              << "  Q/E : arc left / right\n"
              << "  Space: stop\n"
              << "  X   : exit\n";

    std::signal(SIGINT, handle_signal);

    auto robot = std::make_shared<robot_control::DifferentialDriveSim>();
    robot_control::ControlLoop controller(robot, 50.0);

    print_help();
    controller.start();
    enable_raw_mode();

    while (g_running) {
        char key = 0;
        if (read(STDIN_FILENO, &key, 1) > 0) {
            if (key == 'x' || key == 'X') {
                g_running = false;
                break;
            }
            if (key == ' ') {
                controller.set_velocity({});
            } else {
                controller.set_velocity(key_to_twist(key));
            }
        }

        print_state(controller.state());
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    controller.set_velocity({});
    controller.stop();
    restore_terminal();
    std::cout << "\nStopped.\n";
    return 0;
}
