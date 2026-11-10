#ifndef GOTO_XY_H
#define GOTO_XY_H

#include "AttentionalBehavior.h"
#include "RemoteAPIObjects.h"

class GoToXY : public AttentionalBehavior {
private:
    RemoteAPIObjects* sim_;
    int64_t robotHandle_ = -1;

    double targetX_ = 0.0;
    double targetY_ = 0.0;
    
    double currentDistance_ = 999.0;
    double currentAngleError_ = 0.0;
    bool isValidTarget_ = false;

    static constexpr double kWheelBase = 0.331;
    static constexpr double kMapLimit  = 7.5;
    static constexpr double kStopDist  = 0.55;

public:
    GoToXY(const std::string& name, const BT::NodeConfig& config, RemoteAPIObjects* sim);

    static BT::PortsList providedPorts();

    bool checkInternalReleaser() override;
    bool checkExternalReleaser() override;
    void updatePerceptualSchema() override;
    std::pair<double, double> executeMotorSchema() override;
};

#endif // GOTO_XY_H