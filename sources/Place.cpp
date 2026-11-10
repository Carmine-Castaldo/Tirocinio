#include "Place.h"
#include "SharedPerception.h"
#include <algorithm>

Place::Place(const std::string& name, const BT::NodeConfig& config, RemoteAPIObjects* sim)
    : AttentionalBehavior(name, config), sim_(sim) {}

BT::PortsList Place::providedPorts() {
    BT::PortsList ports = {
        BT::InputPort<std::string>("Object"),
        BT::InputPort<std::string>("Color"),
        BT::InputPort<std::string>("Target"),
        BT::BidirectionalPort<bool>("Done"),
        BT::BidirectionalPort<std::vector<int>>("knapsack")
    };
    auto basePorts = AttentionalBehavior::providedPorts();
    ports.insert(basePorts.begin(), basePorts.end());
    return ports;
}

bool Place::checkInternalReleaser() {
    if (targetHandle_ == -1) return false;

    const bool inKnapsack = std::find(knapsack_.begin(), knapsack_.end(),
                                      static_cast<int>(targetHandle_)) != knapsack_.end();
    if (target_ == "ground") {
        return inKnapsack;
    }
    return knapsack_.size() < 4 && !inKnapsack;
}

bool Place::checkExternalReleaser() {
    if (startTime_ >= 0.0) return true;
    bool isDone = false;
    getInput("Done", isDone);
    if (target_ == "knapsack" && isDone) return false;

    return distance_ < kMaxDistance;
}

void Place::updatePerceptualSchema() {
    getInput("Target", target_);
    getInput("Object", objectPath_);
    getInput("knapsack", knapsack_);

    if (!objectPath_.empty() && objectPath_ != previousPath_) {
        targetHandle_ = sp.handle(objectPath_);
        previousPath_ = objectPath_;
    }

    distance_ = 999.0;
    try {
        distance_ = sp.distanceTo(target_ == "ground" ? "/Bill" : objectPath_);
    } catch (...) {}

    if (startTime_ >= 0.0 || distance_ < kMaxDistance) {
        lambda_ = kMinLambda;
    } else {
        lambda_ = saturatingLambda(distance_, 0.5, 1.0);
    }
}

std::pair<double, double> Place::executeMotorSchema() {
    const double now = sp.time();
    if (startTime_ < 0.0)
        startTime_ = now;
    

    if (now - startTime_ < kActionSeconds) {
        return {0.0, 0.0};
    }

    completeAction();
    finished_ = true;
    return {0.0, 0.0};
}

void Place::completeAction() {
    if (target_ == "knapsack") {
        const size_t n = knapsack_.size();
        const int index = static_cast<int>(n) + 6;
        const std::string connectionPath = "/PioneerP3DX/connection[" + std::to_string(index) + "]";

        try {
            const int64_t connHandle = sp.handle(connectionPath);
            sp.exec([&](RemoteAPIObjects& s) {
                s.sim().setObjectInt32Param(targetHandle_, s.sim().shapeintparam_static, 1);
                s.sim().setObjectInt32Param(targetHandle_, s.sim().shapeintparam_respondable, 0);
                s.sim().resetDynamicObject(targetHandle_);
                s.sim().setObjectParent(targetHandle_, connHandle, false);
                s.sim().setObjectPosition(targetHandle_, connHandle, {0.0, 0.0, 0.05});
            });

            if (std::find(knapsack_.begin(), knapsack_.end(), static_cast<int>(targetHandle_)) == knapsack_.end()) {
                knapsack_.push_back(static_cast<int>(targetHandle_));
            }
            setOutput("Done", true);
            setOutput("knapsack", knapsack_);
        } catch (...) {}

    } else if (target_ == "ground") {
        const auto robotPos = sp.position();
        const std::vector<double> groundPos = {robotPos[0] + 0.4, robotPos[1] + 0.3, 0.05};

        try {
            sp.exec([&](RemoteAPIObjects& s) {
                s.sim().setObjectParent(targetHandle_, -1, true);
                s.sim().setObjectPosition(targetHandle_, -1, groundPos);
                s.sim().setObjectInt32Param(targetHandle_, s.sim().shapeintparam_static, 0);
                s.sim().setObjectInt32Param(targetHandle_, s.sim().shapeintparam_respondable, 1);
                s.sim().resetDynamicObject(targetHandle_);
            });

            auto it = std::find(knapsack_.begin(), knapsack_.end(), static_cast<int>(targetHandle_));
            if (it != knapsack_.end())
                knapsack_.erase(it);
            
            setOutput("Done", false);
            setOutput("knapsack", knapsack_);
        } catch (...) {}
    }
}

void Place::afterPropose(){
    int64_t goalHandle = targetHandle_;
    if (target_ == "ground") 
        goalHandle = sp.handle("/Bill");
    if (goalHandle >= 0)
        sp.offerGoal(UID(), goalHandle, emphasis_);
}