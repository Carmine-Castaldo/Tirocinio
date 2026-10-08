#ifndef GOTO_XY_H
#define GOTO_XY_H

#include "AttentionalBehavior.h"
#include "RemoteAPIObjects.h"

class GoToXY : public AttentionalBehavior {
private:
    RemoteAPIObjects* simObj_;
    double targetX_ = 0.0;
    double targetY_ = 0.0;

public:
    GoToXY(const std::string& name, const BT::NodeConfig& config, RemoteAPIObjects* simObj);

    static BT::PortsList providedPorts();

    bool checkReleaser() override;
    void updateClockPeriod() override;
    std::pair<double, double> executeMotorSchema() override;
};

#endif // GOTO_XY_H