#include "ReachColor.h"
#include "SharedPerception.h" 
#include <iostream>
#include <cmath>
#include <algorithm>

ReachColor::ReachColor(const std::string& name, const BT::NodeConfig& config, RemoteAPIObjects* sim)
    : AttentionalBehavior(name, config), sim_(sim) {
    robotHandle_ = sim_->sim().getObjectHandle("/PioneerP3DX");
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
    distance_ = 999.0;
    lambda_   = 1.0;

    getInput("ObjectPath", target_);
    getInput("Knapsack", knapsack);
    if (target_.empty()) return;

    bool taken = false;
    getInput("Done", taken);                       // Done = "questo cubo è stato preso"
    if (target_ == "/Bill") {
        if (!taken || knapsack.empty()) return;    // consegno solo se ho preso QUESTO cubo
    } else {
        if (taken) return;
        if (isObjectInKnapsack()) return;
    }

    try {
       
        targetHandle_ = resolveTarget();
        auto pos = sim_->sim().getObjectPosition(targetHandle_, robotHandle_);;
        relX_ = pos[0]; 
        relY_ = pos[1];
        distance_ = std::hypot(relX_, relY_);
        lambda_ = saturatingLambda(distance_, 1.0, 10.0);
    } catch (...) {
        distance_ = 999.0;
        lambda_ = 1.0;
    }
}

bool ReachColor::checkInternalReleaser() {
    if (distance_ > 10.0 ) return false;
    return true;
}


bool ReachColor::checkExternalReleaser() {
    bool isDone = false;
    getInput("Done", isDone);
    
    if (target_ == "/Bill") 
        return isDone && !knapsack.empty();

    if (isDone || isObjectInKnapsack()) return false; 

    return true;
}


int64_t ReachColor::resolveTarget() {
    if (target_ != previousTarget_) {
        targetHandle_ = sim_->sim().getObjectHandle(target_);
        previousTarget_ = target_;
    }
    return targetHandle_;
}

bool ReachColor::isObjectInKnapsack(){
    try {
        targetHandle_ = resolveTarget();
        int targetHandleInt = static_cast<int>(targetHandle_);
        return (std::find(knapsack.begin(), knapsack.end(), targetHandleInt) != knapsack.end());
    } catch (...) {
        return false;
    }
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
