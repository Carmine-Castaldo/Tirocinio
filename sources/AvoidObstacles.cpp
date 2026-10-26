#include "AvoidObstacles.h"
#include "SharedPerception.h"
#include <algorithm>
#include <cmath>

AvoidObstacles::AvoidObstacles(const std::string& name, const BT::NodeConfig& config, RemoteAPIObjects* sim)
    : AttentionalBehavior(name, config), sim_(sim) {
    robotHandle_ = sim_->sim().getObjectHandle("/PioneerP3DX");

    for (int i = 1; i <= 6; ++i) {
        int64_t h = sim_->sim().getObjectHandle(
            "/PioneerP3DX/ultrasonicSensor[" + std::to_string(i) + "]");
        sonarHandles_.push_back(h);

        auto p = sim_->sim().getObjectPosition(h, robotHandle_);
        double a = std::atan2(p[1], p[0]);
        sonarSide_.push_back((a > 0.35) ? +1 : (a < -0.35 ? -1 : 0));
    }
    lastDistances_.assign(sonarHandles_.size(), -1.0);
}

BT::PortsList AvoidObstacles::providedPorts() {
    BT::PortsList ports = {
        BT::InputPort<std::string>("Object")
    };
    auto basePorts = AttentionalBehavior::providedPorts();
    ports.insert(basePorts.begin(), basePorts.end());
    return ports;
}

bool AvoidObstacles::isIgnored(int64_t detected) {
    if (detected == -1) return false;
    if (detected == targetHandle_) return true;
    if (!ignoredValid_) {
        ignored_.clear();
        try {
            for (auto h : sim_->sim().getObjectsInTree(robotHandle_)) ignored_.insert(h);
        } catch (...) {}
        ignoredValid_ = true;
    }
    return ignored_.count(detected) > 0;
}

double AvoidObstacles::checkSonar(int64_t h) {
    const auto& r = SharedPerception::get().sonar(h);
    if (std::get<0>(r) == 1) {
        int64_t det = std::get<3>(r);
        if (!SharedPerception::get().isGoal(det) &&
            !SharedPerception::get().inRobotTree(det))
            return std::get<1>(r);
    }
    return -1;
}

double AvoidObstacles::readSensors() {
    std::string path;
    getInput("Object", path);
    if (path != previousPath_) {
        targetHandle_ = -1;
        if (path.empty())
            previousPath_ = path;
        else {
            try { targetHandle_ = sim_->sim().getObjectHandle(path); previousPath_ = path; }
            catch (...) {}
        }
    }
    ignoredValid_ = false;
    distances.assign(sonarHandles_.size(), -1.0);
    for (size_t i = 0; i < sonarHandles_.size(); ++i)
        distances[i] = checkSonar(sonarHandles_[i]);

    isObstacle = false;
    double minDistance = 999.0;
    for (double d : distances) {
        if (d > 0.0) {
            isObstacle = true;
            if (d < minDistance) minDistance = d;
        }
    }

    const double t = SharedPerception::get().time();
    if (isObstacle) { 
        holdUntil_ = t + kHoldSec; 
        lastDistances_ = distances; 
        lastMinDist_ = minDistance; 
    } else if (t >= holdUntil_) {
        turnSign_ = 0;
        lastMinDist_ = 999.0; 
    }

    return isObstacle ? minDistance : lastMinDist_;
}

bool AvoidObstacles::checkInternalReleaser() {
    return isObstacle || SharedPerception::get().time() < holdUntil_;
}

std::pair<double,double> AvoidObstacles::executeMotorSchema() {
    const auto& d = isObstacle ? distances : lastDistances_;

    double turn = 0.0, minDist = 999.0, minCenter = 999.0, leftSum = 0.0, rightSum = 0.0;
    for (size_t i = 0; i < d.size(); ++i) {
        if (d[i] <= 0.0) continue;
        minDist = std::min(minDist, d[i]);
        double w = 1.0 / (d[i] + 0.20);
        if (sonarSide_[i] > 0)      { turn -= w; leftSum  += w; }
        else if (sonarSide_[i] < 0) { turn += w; rightSum += w; }
        else                        { minCenter = std::min(minCenter, d[i]); }
    }

    bool symmetric = std::abs(turn) < 0.3 * std::max(leftSum, rightSum);
    if (minCenter < 999.0 || symmetric) {
        if (turnSign_ == 0) turnSign_ = (leftSum > rightSum) ? -1 : +1;
        turn += turnSign_ * (1.0 / (std::min(minCenter, minDist) + 0.20));
    }

    double v = 0.35;
    if (minDist < 0.20)      v = 0.05;
    else if (minDist < 0.40) v = 0.18;

    double w_gain = (minDist < 0.35) ? 0.55 : 0.35;
    double w = std::clamp(turn * w_gain, -1.2, 1.2);

    const double L = 0.331;
    return { v - w * L / 2.0, v + w * L / 2.0 };
}