#include "ReachColor.h"
#include <iostream>
#include <cmath>
#include <algorithm>

ReachColor::ReachColor(const std::string& name, const BT::NodeConfig& config, RemoteAPIObjects* sim)
    : AttentionalBehavior(name, config), sim_(sim) {
    setMu(2.0);
    robotHandle_ = sim_->sim().getObjectHandle("/PioneerP3DX");
    drawHandle_ = -1;
}

BT::PortsList ReachColor::providedPorts() {
    BT::PortsList ports = {
        BT::InputPort<std::string>("Color"),
        BT::InputPort<std::string>("ObjectPath"),
        BT::InputPort<std::vector<int>>("Knapsack"),
        BT::InputPort<bool>("Done"),
        BT::InputPort<bool>("Return")
    };
    auto basePorts = AttentionalBehavior::providedPorts();
    ports.insert(basePorts.begin(), basePorts.end());
    return ports;
}

void ReachColor::updatePerceptualSchema() {
    getInput("ObjectPath", target_);
    getInput("Knapsack", knapsack);

    if (target_.empty()) {
        distance_ = 999.0;
        lambda_ = 20.0;
        return;
    }

    try {
        int64_t th = sim_->sim().getObjectHandle(target_);
        auto pos = sim_->sim().getObjectPosition(th, robotHandle_);
        relX_ = pos[0]; 
        relY_ = pos[1];
        distance_ = std::hypot(relX_, relY_);
        lambda_ = std::max(0.1, distance_);
    } catch (...) {
        distance_ = 999.0;
        lambda_ = 20.0;
    }
}

bool ReachColor::checkInternalReleaser() {
    if (distance_ > 10.0 || distance_ < 0.45) return false;
    return true;
}

bool isObjectInKnapsack(RemoteAPIObjects* sim_, std::string target_, std::vector<int> knapsack) {
    try {
        int64_t targetHandle = sim_->sim().getObjectHandle(target_);
        int targetHandleInt = static_cast<int>(targetHandle);
        return (std::find(knapsack.begin(), knapsack.end(), targetHandleInt) != knapsack.end());
    } catch (...) {
        return false;
    }
}

bool isKnapsackFull(std::vector<int> knapsack) {
    return (knapsack.size() > 3);
}

bool ReachColor::checkExternalReleaser() {
    bool isDone = false;
    getInput("Done", isDone);
    if (isDone) return false;

    if (target_ == "/Bill") {
        if (!knapsack.empty()) return true; 
        return false;
    }

    if (isObjectInKnapsack(sim_, target_, knapsack)) return false; 

    return true;
}

std::pair<double, double> ReachColor::executeMotorSchema() {
    

    if (distance_ <= 0.55) return {0.0, 0.0};

    double angle = std::atan2(relY_, relX_);

    double v_max = 0.40;
    if (distance_ < 1.0) {
        v_max = 0.15 + (distance_ / 1.0) * 0.25;
    }
    
    double v = 0.0;
    if (std::abs(angle) < (M_PI / 3.0)) {
        v = v_max * std::cos(angle);
    }

    double w = std::clamp(angle * 1.8, -1.2, 1.2);

    double L = 0.331;
    double v_left  = v - (w * L) / 2.0;
    double v_right = v + (w * L) / 2.0;

    return {v_left, v_right};
}
