#ifndef MOTOR_COMMAND_H
#define MOTOR_COMMAND_H

#include <string>
#include <utility>

struct MotorCommand {
    std::string owner = "";
    double emphasis = -1.0;
    std::pair<double, double> command = {0.0, 0.0};

};

#endif // MOTOR_COMMAND_H