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
    int64_t robotHandle_ = -1;
    int64_t targetHandle_ = -1;
    std::string previousPath_;
    

    std::vector<int> sonarSide_;
    std::vector<double>  distances_;
    std::vector<double>  lastDistances_;
    std::vector<int64_t> sonarHandles_;

    int    turnSign_ = 0;
    bool   isObstacle_ = false;
    double holdUntil_ = 0.0;
    double lastMinDist_ = 1.0;
    

    static constexpr double kHoldSec = 0.5;
    static constexpr double kRobotRadius = 0.20;
    static constexpr double kWheelBase = 0.331;

    
    double checkSonar(int64_t sonarHandle);
    void setupSensors();
    void updateTargetHandle();

public:
    AvoidObstacles(const std::string& name, const BT::NodeConfig& config, RemoteAPIObjects* sim);

    static BT::PortsList providedPorts();

    bool checkInternalReleaser() override;
    bool checkExternalReleaser() override;
    void updatePerceptualSchema() override;
    std::pair<double, double> executeMotorSchema() override;
    
    bool exclusive() const override { return false; } 
};

#endif // AVOID_OBSTACLES_H