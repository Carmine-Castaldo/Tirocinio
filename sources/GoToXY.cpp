#include "GoToXY.h"
#include <iostream>
#include <cmath>
#include <algorithm>

GoToXY::GoToXY(const std::string& name, const BT::NodeConfig& config, RemoteAPIObjects* sim)
        : AttentionalBehavior(name, config), sim_(sim) {
            setMu(0.5);
            robotHandle_ = sim_->sim().getObjectHandle("/PioneerP3DX");
        }

BT::PortsList GoToXY::providedPorts() {
    BT::PortsList ports = { 
        BT::InputPort<double>("X"), 
        BT::InputPort<double>("Y"),
        BT::InputPort<std::string>("TargetID")
    };
    auto basePorts = AttentionalBehavior::providedPorts();
    ports.insert(basePorts.begin(), basePorts.end());
    return ports;
}
void GoToXY::updatePerceptualSchema() {
    if (!getInput("X", targetX_) || !getInput("Y", targetY_)) {
        dist_ = 999.0;
        lambda_ = 20.0;
        return;
    }

    try {
        auto robotPos = sim_->getRobotPosition();
        double dx = targetX_ - robotPos[0];
        double dy = targetY_ - robotPos[1];
        dist_ = std::sqrt(dx * dx + dy * dy);

        if (dist_ <= 0.5) lambda_ = 1.0;
        else if (dist_ >= 10.0) lambda_ = 10.0;
        else lambda_ = 1.0 + 0.9 * (dist_ - 1.0);

        std::vector<double> ori = sim_->sim().getObjectOrientation(robotHandle_, -1);
        double yaw = ori[2];
        double targetAngle = std::atan2(dy, dx);
        angleError_ = std::atan2(std::sin(targetAngle - yaw), std::cos(targetAngle - yaw));

        // std::cout << "[GoToXY] Target: (" << targetX_ << ", " << targetY_
        //           << ") | Dist: " << dist_ << " | Lambda: " << lambda_
        //           << " mu: " << getMu() << std::endl;

    } catch (const std::exception& e) {
        dist_ = 999.0;
        lambda_ = 10.0;
    } catch (...) {
        dist_ = 999.0;
        lambda_ = 10.0;
    }
}

bool GoToXY::checkInternalReleaser() {
    if (std::abs(targetX_) > 7.5 || std::abs(targetY_) > 7.5) {
        return false;
    }
    return true;
}

bool GoToXY::checkExternalReleaser() {
    return true;
}

std::pair<double, double> GoToXY::executeMotorSchema() {
    if (dist_ <= 0.45) return {0.0, 0.0};

    double v = 0.25 * std::max(0.0, std::cos(angleError_));
    double w = std::clamp(angleError_ * 1.0, -0.8, 0.8);

    double v_left  = v - w * 0.331 / 2.0;
    double v_right = v + w * 0.331 / 2.0;
    return {v_left, v_right};
}