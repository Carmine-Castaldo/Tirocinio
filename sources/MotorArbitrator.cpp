#include "MotorArbitrator.h"
#include "SharedPerception.h"
#include <cmath>
#include <iostream>

MotorArbitrator::MotorArbitrator(const std::string& name, const BT::NodeConfig& config)
    : BT::ActionNodeBase(name, config) {
    auto& sp    = SharedPerception::get();
    leftMotor_  = sp.handle("/PioneerP3DX/leftMotor");
    rightMotor_ = sp.handle("/PioneerP3DX/rightMotor");
}

BT::PortsList MotorArbitrator::providedPorts() {
    return {
        BT::BidirectionalPort<MotorCommand>("Motor")
    };
}

void MotorArbitrator::halt() {
    previousWinner_ = "none";
    prevVL_         = 1e9;
    prevVR_         = 1e9;
    resetStatus();
}

MotorArbitrator::ArbitratedOutput MotorArbitrator::resolveCommand() {
    ArbitratedOutput out;

    MotorCommand cmd;
    if (!getInput("Motor", cmd)) {
        return out;
    }

    const bool hasWinner = !cmd.owner.empty() && cmd.emphasis > 0.0;
    const bool hasAvoid  = !cmd.avoidOwner.empty() && cmd.avoidEmphasis > 0.0;

    if (hasWinner && hasAvoid) {
        const double totalEmphasis = cmd.emphasis + cmd.avoidEmphasis;
        const double avoidRatio    = cmd.avoidEmphasis / totalEmphasis;
        const double goalRatio     = 1.0 - avoidRatio;

        out.vL       = (cmd.command.first  * goalRatio) + (cmd.avoidVL * avoidRatio);
        out.vR       = (cmd.command.second * goalRatio) + (cmd.avoidVR * avoidRatio);
        out.winner   = cmd.owner + "+avoid";
        out.emphasis = cmd.emphasis;
    } else if (hasWinner) {
        out.vL       = cmd.command.first;
        out.vR       = cmd.command.second;
        out.winner   = cmd.owner;
        out.emphasis = cmd.emphasis;
    } else if (hasAvoid) {
        out.vL       = cmd.avoidVL;
        out.vR       = cmd.avoidVR;
        out.winner   = "avoid";
        out.emphasis = cmd.avoidEmphasis;
    }

    return out;
}

void MotorArbitrator::applyToSimulator(const ArbitratedOutput& out) {
    const bool stopping   = out.vL == 0.0 && out.vR == 0.0 && (prevVL_ != 0.0 || prevVR_ != 0.0);
    const bool velChanged = stopping ||
                            std::abs(out.vL - prevVL_) > kVelEpsilon ||
                            std::abs(out.vR - prevVR_) > kVelEpsilon;
    const bool winChanged = out.winner != previousWinner_;

    if (!velChanged && !winChanged) {
        return;
    }

    SharedPerception::get().exec([&](RemoteAPIObjects& s) {
        try {
            if (velChanged) {
                s.sim().setJointTargetVelocity(leftMotor_,  out.vL / kWheelRadius);
                s.sim().setJointTargetVelocity(rightMotor_, out.vR / kWheelRadius);
            }
            if (winChanged) {
                s.sim().setStringSignal("active_behavior", out.winner);
            }
        } catch (const std::exception& e) {
            std::cout << "[MotorArbitrator] error: " << e.what() << '\n';
        }
    });

    prevVL_         = out.vL;
    prevVR_         = out.vR;
    previousWinner_ = out.winner;
}

BT::NodeStatus MotorArbitrator::tick() {
    const ArbitratedOutput out = resolveCommand();
    applyToSimulator(out);
    return BT::NodeStatus::RUNNING;
}