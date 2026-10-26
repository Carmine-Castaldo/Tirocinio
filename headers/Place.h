#pragma once
#include "AttentionalBehavior.h"
#include "RemoteAPIObjects.h"
#include <string>

class Place : public AttentionalBehavior {
private:
    RemoteAPIObjects* sim_;
    std::string objectPath_, previousPath_, target_, color_;
    int64_t targetHandle_ = -1;
    double distance_ = 999.0;
    std::vector<int> knapsack;

public:
    Place(const std::string& name, const BT::NodeConfig& config, RemoteAPIObjects* sim);

    static BT::PortsList providedPorts();

    double readSensors() override;
    bool checkInternalReleaser() override;
    std::pair<double, double> executeMotorSchema() override;
};