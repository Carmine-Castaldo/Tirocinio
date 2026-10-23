#ifndef GOTO_XY_H
#define GOTO_XY_H

#include "AttentionalBehavior.h"
#include "RemoteAPIObjects.h"

class GoToXY : public AttentionalBehavior {
private:
    RemoteAPIObjects* sim_;
    double targetX_ = 0.0;
    double targetY_ = 0.0;
    int64_t robotHandle_ = -1;
    double dist_ = 999.0, angleError_ = 0.0;

public:
    GoToXY(const std::string& name, const BT::NodeConfig& config, RemoteAPIObjects* sim);

    static BT::PortsList providedPorts();

    bool checkInternalReleaser() override;
    bool checkExternalReleaser() override;
    void updatePerceptualSchema() override;
    std::pair<double, double> executeMotorSchema() override;
    bool accomplishesTask() const override { return true; }
};

#endif // GOTO_XY_H