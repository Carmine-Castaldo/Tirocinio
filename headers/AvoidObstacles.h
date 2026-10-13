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
    int64_t targetHandle_, robotHandle_;
    std::vector<double> distances;
    bool isObstacle;    

public:
    AvoidObstacles(const std::string& name, const BT::NodeConfig& config, RemoteAPIObjects* sim);

    static BT::PortsList providedPorts();

    bool checkInternalReleaser() override;
    bool checkExternalReleaser() override;
    void updatePerceptualSchema() override;
    std::pair<double, double> executeMotorSchema() override;
    double checkSonar(int64_t sonarHandle);
};


#endif // AVOID_OBSTACLES_H