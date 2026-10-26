#include "Place.h"
#include "SharedPerception.h"
#include <iostream>

Place::Place(const std::string& name, const BT::NodeConfig& config, RemoteAPIObjects* sim)
    : AttentionalBehavior(name, config), sim_(sim) {}

BT::PortsList Place::providedPorts() {
    BT::PortsList ports = {
        BT::InputPort<std::string>("Object"),
        BT::InputPort<std::string>("Target"),
        BT::BidirectionalPort<std::vector<int>>("knapsack"),
        BT::InputPort<std::string>("Color")
    };
    auto basePorts = AttentionalBehavior::providedPorts();
    ports.insert(basePorts.begin(), basePorts.end());
    return ports;
}

double Place::readSensors() {
    getInput("Target", target_);
    getInput("Object", objectPath_);
    getInput("knapsack", knapsack);
    getInput("Color", color_);
    if (!objectPath_.empty() && objectPath_ != previousPath_) {
        targetHandle_ = sim_->sim().getObjectHandle(objectPath_);
        previousPath_ = objectPath_;
    }
    distance_ = 999.0;
    try { distance_ = sim_->getDistanceTo(target_ == "ground" ? "/Bill" : objectPath_); }
    catch (...) {}

    return distance_;
}


bool Place::checkInternalReleaser() {
    return (targetHandle_ != -1 && knapsack.size() < 4 && distance_ < 0.6);
}

std::pair<double, double> Place::executeMotorSchema() {
    if (target_ == "knapsack") {
        try {
            const size_t n = knapsack.size();
            int64_t connHandle = sim_->sim().getObjectHandle(
                "/PioneerP3DX/connection[" + std::to_string(n + 6) + "]");
            sim_->sim().setObjectInt32Param(targetHandle_, sim_->sim().shapeintparam_static, 1);
            sim_->sim().setObjectInt32Param(targetHandle_, sim_->sim().shapeintparam_respondable, 0);
            sim_->sim().resetDynamicObject(targetHandle_);
            sim_->sim().setObjectParent(targetHandle_, connHandle, false);
            sim_->sim().setObjectPosition(targetHandle_, connHandle, {0.0, 0.0, 0.05});
            if (std::find(knapsack.begin(), knapsack.end(), static_cast<int>(targetHandle_)) == knapsack.end())
                knapsack.push_back(static_cast<int>(targetHandle_));
            setOutput("knapsack", knapsack);
        } catch (...) {}
    } else if (target_ == "ground") {
        sim_->sim().setObjectParent(targetHandle_, -1, true);
        auto robotPos = SharedPerception::get().position();
        
        std::vector<double> groundPos = {robotPos[0] + 0.4, robotPos[1] + 0.3, 0.05};
        sim_->sim().setObjectPosition(targetHandle_, -1, groundPos);
        
        sim_->sim().setObjectInt32Param(targetHandle_, sim_->sim().shapeintparam_static, 0);
        sim_->sim().setObjectInt32Param(targetHandle_, sim_->sim().shapeintparam_respondable, 1);
        sim_->sim().resetDynamicObject(targetHandle_);

        auto it = std::find(knapsack.begin(), knapsack.end(), static_cast<int>(targetHandle_));
        if (it != knapsack.end())
            knapsack.erase(it);
        
        setOutput("knapsack", knapsack);
    }

    return {0.0, 0.0};
}