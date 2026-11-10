#include "AvoidObstacles.h"
#include "SharedPerception.h"
#include <algorithm>
#include <cmath>

AvoidObstacles::AvoidObstacles(const std::string& name, const BT::NodeConfig& config, RemoteAPIObjects* sim)
    : AttentionalBehavior(name, config), sim_(sim) {
    setupSensors();
}

BT::PortsList AvoidObstacles::providedPorts() {
    BT::PortsList ports = {
        BT::InputPort<std::string>("Object")
    };
    auto basePorts = AttentionalBehavior::providedPorts();
    ports.insert(basePorts.begin(), basePorts.end());
    return ports;
}

void AvoidObstacles::setupSensors() {
    auto& sp = SharedPerception::get();
    robotHandle_ = sp.handle("/PioneerP3DX");

    sonarHandles_.clear();
    sonarSide_.clear();
    for (int i = 1; i <= 6; ++i) {
        const int64_t h = sp.handle("/PioneerP3DX/ultrasonicSensor[" + std::to_string(i) + "]");
        sonarHandles_.push_back(h);
        sonarSide_.push_back(sp.sonarSide(h));
    }

    distances_.assign(sonarHandles_.size(), -1.0);
    lastDistances_.assign(sonarHandles_.size(), -1.0);
}

double AvoidObstacles::checkSonar(int64_t h) {
    const auto& result = SharedPerception::get().sonar(h);
    const int state = std::get<0>(result);
    
    if (state == 1) {
        const double dist = std::get<1>(result);
        const int64_t detectedObj = std::get<3>(result);
        
        if (!SharedPerception::get().isGoal(detectedObj) &&
            !SharedPerception::get().inRobotTree(detectedObj)) {
            return dist;
        }
    }
    return -1.0;
}

void AvoidObstacles::updateTargetHandle() {
    std::string currentPath;
    if (getInput("Object", currentPath) && currentPath != previousPath_) {
        targetHandle_ = -1;
        previousPath_ = currentPath;
        if (!currentPath.empty()) {
            try { targetHandle_ = SharedPerception::get().handle(currentPath);}
            catch (...) {}
        }
    }
}

void AvoidObstacles::updatePerceptualSchema() {
    updateTargetHandle();
    
    double minDistance = 999.0;
    isObstacle_ = false;

    for (size_t i = 0; i < sonarHandles_.size(); ++i) {
        distances_[i] = checkSonar(sonarHandles_[i]);
        if (distances_[i] > 0.0) {
            isObstacle_ = true;
            minDistance = std::min(minDistance, distances_[i]);
        }
    }

    const double now = SharedPerception::get().time();
    
    if (isObstacle_) {
        holdUntil_ = now + kHoldSec;
        lastDistances_ = distances_;
        lastMinDist_ = minDistance;
    } else if (now >= holdUntil_)
        turnSign_ = 0;
    
    double effectiveDist = isObstacle_ ? minDistance : lastMinDist_;
    lambda_ = saturatingLambda(effectiveDist, 0.5, 1.0);
}

bool AvoidObstacles::checkInternalReleaser() {
    return isObstacle_ || SharedPerception::get().time() < holdUntil_;
}

bool AvoidObstacles::checkExternalReleaser() {
    return true;
}

std::pair<double, double> AvoidObstacles::executeMotorSchema() {
    const auto& d = isObstacle_ ? distances_ : lastDistances_;

    double turn = 0.0;
    double minDist = 999.0;
    double minCenterDist = 999.0;
    double leftSum = 0.0;
    double rightSum = 0.0;

    for (size_t i = 0; i < d.size(); ++i) {
        if (d[i] <= 0.0) continue;
        
        minDist = std::min(minDist, d[i]);
        double weight = 1.0 / (d[i] + kRobotRadius); 
        
        if (sonarSide_[i] > 0) { 
            turn -= weight; 
            leftSum += weight; 
        } else if (sonarSide_[i] < 0) { 
            turn += weight; 
            rightSum += weight; 
        } else
            minCenterDist = std::min(minCenterDist, d[i]); 
    }

    bool isSymmetric = std::abs(turn) < 0.3 * std::max(leftSum, rightSum);
    if (minCenterDist < 999.0 || isSymmetric) {
        if (turnSign_ == 0)
            turnSign_ = (leftSum > rightSum) ? -1 : +1;
        
        turn += turnSign_ * (1.0 / (std::min(minCenterDist, minDist) + kRobotRadius));
    }

    double linearVel = 0.175;
    double w_gain = 0.175;

    if (minDist < 0.20) {
        linearVel = 0.025;
        w_gain = 0.275;
    } else if (minDist < 0.40) 
        linearVel = 0.09;

    double w = std::clamp(turn * w_gain, -1.2, 1.2); 
    
    double v_left  = linearVel - (w * kWheelBase) / 2.0;
    double v_right = linearVel + (w * kWheelBase) / 2.0;

    return {v_left, v_right};
}