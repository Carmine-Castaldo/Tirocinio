#pragma once
#include "AttentionalBehavior.h"
#include "RemoteAPIObjects.h"
#include <string>

class GoToXY : public AttentionalBehavior {
private:
    RemoteAPIObjects* sim_;
    int64_t robotHandle_ = -1;

    double targetX_ = 0.0;
    double targetY_ = 0.0;
    double dist_ = 999.0;
    double angleError_ = 0.0;

public:
    GoToXY(const std::string& name, const BT::NodeConfig& config, RemoteAPIObjects* sim);

    static BT::PortsList providedPorts();

    double readSensors() override;
    bool checkInternalReleaser() override;
    std::pair<double, double> executeMotorSchema() override;
};