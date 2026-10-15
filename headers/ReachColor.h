#ifndef REACH_COLOR_H
#define REACH_COLOR_H

#include "AttentionalBehavior.h"
#include "SharedPerception.h"
#include "RemoteAPIObjects.h"
#include <string>
#include <cmath>
#include <algorithm>

class ReachColor : public AttentionalBehavior {
private:
    RemoteAPIObjects* sim_;
    std::string color_, target_, previousTarget_;
    std::vector<int> knapsack;
    double distance_, relX_ = 0, relY_ = 0;
    int64_t robotHandle_ = -1, targetHandle_ = -1;
    int64_t resolveTarget();
    bool isObjectInKnapsack();

public:
    ReachColor(const std::string& name, const BT::NodeConfig& config, RemoteAPIObjects* sim);

    static BT::PortsList providedPorts();

    bool checkInternalReleaser() override;
    bool checkExternalReleaser() override;
    void updatePerceptualSchema() override;
    void afterPropose() override { SharedPerception::get().offerGoal(targetHandle_, emphasis_); }
    std::pair<double, double> executeMotorSchema() override;
};

#endif // REACH_COLOR_H