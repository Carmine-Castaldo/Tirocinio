#include "GoToXY.h"
#include "SharedPerception.h"
#include <iostream>
#include <cmath>
#include <algorithm>

GoToXY::GoToXY(const std::string& name, const BT::NodeConfig& config, RemoteAPIObjects* sim) 
    : AttentionalBehavior(name, config), sim_(sim) { 
    robotHandle_ = sim_->sim().getObjectHandle("/PioneerP3DX"); 
}

BT::PortsList GoToXY::providedPorts() { 
    BT::PortsList ports = { 
        BT::InputPort<double>("X"), 
        BT::InputPort<double>("Y"),
        BT::InputPort<std::string>("Object")
    }; 
    auto basePorts = AttentionalBehavior::providedPorts(); 
    ports.insert(basePorts.begin(), basePorts.end()); 
    return ports; 
}

double GoToXY::readSensors() {
    getInput("X", targetX_);
    getInput("Y", targetY_);

    const auto& robotPos = SharedPerception::get().position();
    const auto& robotOri = SharedPerception::get().orientation();

    double dx = targetX_ - robotPos[0];
    double dy = targetY_ - robotPos[1];
    
    dist_ = std::hypot(dx, dy);

    double targetAngle = std::atan2(dy, dx);
    angleError_ = targetAngle - robotOri[2];
    
    while (angleError_ > M_PI)  angleError_ -= 2.0 * M_PI;
    while (angleError_ < -M_PI) angleError_ += 2.0 * M_PI;

    return dist_;
}

bool GoToXY::checkInternalReleaser() { 
    if (std::abs(targetX_) > 7.5 || std::abs(targetY_) > 7.5) return false;     
    return true; 
}

std::pair<double, double> GoToXY::executeMotorSchema() { 
    if (dist_ <= 0.55) return {0.0, 0.0};
    double v = 0.25 * std::max(0.0, std::cos(angleError_));
    double w = std::clamp(angleError_ * 1.0, -0.8, 0.8);

    double L = 0.331;
    double v_left  = v - w * L / 2.0;
    double v_right = v + w * L / 2.0;
    
    return {v_left, v_right};
}