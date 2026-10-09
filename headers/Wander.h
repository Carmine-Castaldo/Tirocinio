#ifndef WANDER_H
#define WANDER_H

#include "AttentionalBehavior.h"
#include "RemoteAPIObjects.h"
#include <random>

class Wander : public AttentionalBehavior {
private:
    RemoteAPIObjects* sim_;
    double targetX_ = 0.0;
    double targetY_ = 0.0;
    double current_dist_ = 999.0;
    int64_t robotHandle_ = -1;
    double angleError_ = 0.0;

    std::mt19937 rng_{std::random_device{}()};
    std::uniform_real_distribution<double> dist_{-6.0, 6.0};
public:
    Wander(const std::string& name, const BT::NodeConfig& config, RemoteAPIObjects* sim);

    static BT::PortsList providedPorts();

    bool checkInternalReleaser() override;
    bool checkExternalReleaser() override;
    void updatePerceptualSchema() override;
    std::pair<double, double> executeMotorSchema() override;
};

#endif // WANDER_H