#include "GoToXY.h"
#include "SharedPerception.h"
#include <iostream>
#include <cmath>
#include <algorithm>

GoToXY::GoToXY(const std::string& name, const BT::NodeConfig& config, RemoteAPIObjects* sim)
    : AttentionalBehavior(name, config), sim_(sim) {
    robotHandle_ = sp.robotHandle();
}

BT::PortsList GoToXY::providedPorts() {
    BT::PortsList ports = { 
        BT::InputPort<std::string>("Object"),
        BT::InputPort<bool>("Done"),
        BT::InputPort<double>("X"), 
        BT::InputPort<double>("Y")
    };
    auto basePorts = AttentionalBehavior::providedPorts();
    ports.insert(basePorts.begin(), basePorts.end());
    return ports;
}

void GoToXY::updatePerceptualSchema() {

    if (!getInput("X", targetX_) || !getInput("Y", targetY_)) {
        currentDistance_ = 999.0;
        isValidTarget_ = false;
        return;
    }
    
    isValidTarget_ = true;

    try {

        const auto& robotPos = SharedPerception::get().position();
        const auto& ori = SharedPerception::get().orientation();
  
    
        double dx = targetX_ - robotPos[0];
        double dy = targetY_ - robotPos[1];
        currentDistance_ = std::hypot(dx, dy);

        double targetAngle = std::atan2(dy, dx);
        double yaw = ori[2];
        
    
        currentAngleError_ = std::atan2(std::sin(targetAngle - yaw), std::cos(targetAngle - yaw));

        lambda_ = saturatingLambda(currentDistance_, 1.0, 10.0);

    } catch (const std::exception& e) {
        currentDistance_ = 999.0;
        lambda_ = 1.0;
    }
}

bool GoToXY::checkInternalReleaser() {
    if (!isValidTarget_) return false;
    if (std::abs(targetX_) > kMapLimit || std::abs(targetY_) > kMapLimit)
        return false;
    
    return true; 
}

 bool GoToXY::checkExternalReleaser() {
    std::string target;
    bool isDone = false;
    getInput("Done", isDone);
    getInput("Object", target);
    if (target == "/Bill") return isDone;
    return !isDone;
} 

std::pair<double, double> GoToXY::executeMotorSchema() {
    if (currentDistance_ <= kStopDist)
        return {0.0, 0.0};

    double v = 0.125 * std::max(0.0, std::cos(currentAngleError_));
    
    double w = std::clamp(currentAngleError_ * 0.5, -0.8, 0.8); 

    double v_left  = v - (w * kWheelBase) / 2.0;
    double v_right = v + (w * kWheelBase) / 2.0;
    
    return {v_left, v_right};
}