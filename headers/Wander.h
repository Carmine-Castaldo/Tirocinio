#ifndef WANDER_H
#define WANDER_H

#include "AttentionalBehavior.h"

class Wander : public AttentionalBehavior {
public:
    Wander(const std::string& name, const BT::NodeConfig& config);

    static BT::PortsList providedPorts();

    bool checkReleaser() override;
    void updateClockPeriod() override;
    std::pair<double, double> executeMotorSchema() override;
};

#endif // WANDER_H