#pragma once
#include "AttentionalBehavior.h"
#include "RemoteAPIObjects.h"
#include <vector>
#include <string>

class ReachColor : public AttentionalBehavior {
private:
    RemoteAPIObjects* sim_;
    int64_t robotHandle_ = -1;
    int64_t targetHandle_ = -1;
    
    std::string target_;
    std::string previousTarget_;
    
    double distance_ = 999.0;
    double relX_ = 0.0;
    double relY_ = 0.0;
    
    bool isTargetVisible_ = false;
    bool tracking_ = false;
    
    int64_t resolveTarget();

public:
    ReachColor(const std::string& name, const BT::NodeConfig& config, RemoteAPIObjects* sim);
    static BT::PortsList providedPorts();
    double readSensors() override;
    bool checkInternalReleaser() override;
    std::pair<double, double> executeMotorSchema() override;
    void afterPropose() override { SharedPerception::get().offerGoal(targetHandle_, emphasis_); }
};