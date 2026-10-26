#ifndef AVOID_OBSTACLES_H
#define AVOID_OBSTACLES_H

#include "AttentionalBehavior.h"
#include "RemoteAPIObjects.h"
#include <vector>
#include <unordered_set>
#include <string>

class AvoidObstacles : public AttentionalBehavior {
private:
    RemoteAPIObjects* sim_;
    std::vector<int64_t> sonarHandles_;
    int64_t targetHandle_ = -1;
    int64_t robotHandle_  = -1;
    std::vector<double> distances;
    std::string previousPath_;
    std::unordered_set<int64_t> ignored_;
    
    bool isObstacle    = false;
    bool ignoredValid_ = false;
    bool isIgnored(int64_t detected);
    
    int turnSign_     = 0;
    double holdUntil_ = 0.0;
    static constexpr double kHoldSec = 1.0;
    
    std::vector<double> lastDistances_;
    std::vector<int> sonarSide_;
    double lastMinDist_ = 1.0;

public:
    AvoidObstacles(const std::string& name, const BT::NodeConfig& config, RemoteAPIObjects* sim);

    static BT::PortsList providedPorts();
    bool checkInternalReleaser() override;
    std::pair<double, double> executeMotorSchema() override;    
    double checkSonar(int64_t sonarHandle);
    bool exclusive() const override { return false; } 
    double readSensors() override;
};

#endif // AVOID_OBSTACLES_H