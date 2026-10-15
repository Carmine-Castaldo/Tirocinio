#ifndef AVOID_OBSTACLES_H
#define AVOID_OBSTACLES_H

#include "AttentionalBehavior.h"
#include "RemoteAPIObjects.h"
#include <vector>

class AvoidObstacles : public AttentionalBehavior {
private:
    RemoteAPIObjects* sim_;
    std::vector<int64_t> sonarHandles_;
    std::string target_;
    int64_t targetHandle_ = -1, robotHandle_;
    std::vector<double> distances;
    std::string previousPath_;
    std::unordered_set<int64_t> ignored_;
    bool isObstacle = false;
    bool ignoredValid_ = false;
    bool isIgnored(int64_t detected);
    int  turnSign_= 0;
    double holdUntil_ = 0.0;
    static constexpr double kHoldSec = 1.0;
    static constexpr int kHold = 20;
    std::vector<double> lastDistances_;
    std::vector<int> sonarSide_;

    double lastMinDist_ = 1.0;

public:
    AvoidObstacles(const std::string& name, const BT::NodeConfig& config, RemoteAPIObjects* sim);

    static BT::PortsList providedPorts();

    bool checkInternalReleaser() override;
    bool checkExternalReleaser() override;
    void updatePerceptualSchema() override;
    std::pair<double, double> executeMotorSchema() override;
    double checkSonar(int64_t sonarHandle);
    bool exclusive() const override {return false;} 
};


#endif // AVOID_OBSTACLES_H