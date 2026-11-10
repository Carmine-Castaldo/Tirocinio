#ifndef MOTORARBITRATOR_H
#define MOTORARBITRATOR_H

#include <behaviortree_cpp/action_node.h>
#include <chrono>
#include <cstdint>
#include <string>

#include "MotorCommand.h"

class MotorArbitrator : public BT::ThreadedAction {
private:
    static constexpr double kWheelRadius = 0.0975;
    static constexpr double kVelEpsilon  = 1e-4;
    static constexpr auto kWorkersTimeout = std::chrono::milliseconds(150);

    int64_t leftMotor_  = -1;
    int64_t rightMotor_ = -1;

    struct ArbitratedOutput {
        std::string winner = "none";
        double vL          = 0.0;
        double vR          = 0.0;
        double emphasis    = -1.0;
    };

    ArbitratedOutput resolveCommand();
    void applyToSimulator(const ArbitratedOutput& out,
                          std::string& previousWinner,
                          double& prevVL,
                          double& prevVR);

public:
    MotorArbitrator(const std::string& name, const BT::NodeConfig& config);
    ~MotorArbitrator() override = default;

    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;
    void halt() override;
};

#endif // MOTORARBITRATOR_H