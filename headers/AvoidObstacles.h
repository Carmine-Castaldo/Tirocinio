#ifndef AVOID_OBSTACLES_H
#define AVOID_OBSTACLES_H

#include "AttentionalBehavior.h"
#include "RemoteAPIObjects.h"
#include <vector>

class AvoidObstacles : public AttentionalBehavior {
private:
    RemoteAPIObjects* simObj_;
    std::vector<int64_t> sonarHandles_;

public:
    AvoidObstacles(const std::string& name, const BT::NodeConfig& config, RemoteAPIObjects* simObj);

    static BT::PortsList providedPorts();

    bool checkReleaser() override;
    void updatePerceptualSchema() override;
    std::pair<double, double> executeMotorSchema() override;
};

#endif // AVOID_OBSTACLES_H