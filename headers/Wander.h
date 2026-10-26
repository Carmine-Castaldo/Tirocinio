#pragma once
#include "AttentionalBehavior.h"
#include "RemoteAPIObjects.h"
#include <random>

class Wander : public AttentionalBehavior {
private:
    RemoteAPIObjects* sim_;
    std::mt19937 gen_;
    std::uniform_real_distribution<double> distDir_;
    
    double currentW_ = 0.0;
    double nextChangeTime_ = 0.0;

public:
    Wander(const std::string& name, const BT::NodeConfig& config, RemoteAPIObjects* sim);

    static BT::PortsList providedPorts();

    double readSensors() override;
    bool checkInternalReleaser() override;
    std::pair<double, double> executeMotorSchema() override;
    
};