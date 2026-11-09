#ifndef MOTOR_COMMAND_H
#define MOTOR_COMMAND_H

#include <string>
#include <utility>
#include <unordered_map>
#include <memory>
#include <mutex>

struct MotorCommand {
    std::string owner = "";
    double emphasis = -1.0;
    std::pair<double, double> command = {0.0, 0.0};

    std::string avoidOwner = "";
    double avoidVL = 0.0;
    double avoidVR = 0.0;
    double avoidEmphasis = -1.0;

    static std::mutex& lock() { static std::mutex m; return m; }
};

#endif // MOTOR_COMMAND_H