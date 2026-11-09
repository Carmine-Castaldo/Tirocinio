#include "MotorArbitrator.h"
#include "SharedPerception.h"
#include "MotorCommand.h"
#include <chrono>
#include <cmath>
#include <iostream>

MotorArbitrator::MotorArbitrator(const std::string& name, const BT::NodeConfig& config)
    : BT::ThreadedAction(name, config) {
    auto& sp = SharedPerception::get();
    leftMotor_  = sp.handle("/PioneerP3DX/leftMotor");
    rightMotor_ = sp.handle("/PioneerP3DX/rightMotor");
}

BT::PortsList MotorArbitrator::providedPorts() {
    return { BT::BidirectionalPort<MotorCommand>("Motor") };
}

void MotorArbitrator::halt() {
    SharedPerception::get().wake();
    BT::ThreadedAction::halt();
}

BT::NodeStatus MotorArbitrator::tick() {
    auto& sp = SharedPerception::get();
    std::string previousWinner = "none";
    double prevVL = 1e9, prevVR = 1e9;
    uint64_t seen = 0;

    while (!isHaltRequested()) {
        if (!sp.waitNextTick(seen, [this] { return isHaltRequested(); })) continue;
        sp.waitWorkersDone(std::chrono::milliseconds(500));

        std::string winner;
        double vL = 0.0, vR = 0.0, emphasis = -1.0;

        {
            std::lock_guard<std::mutex> lock(MotorCommand::lock());
            MotorCommand cmd;
            if (getInput("Motor", cmd)) {
                winner   = cmd.owner;
                emphasis = cmd.emphasis;

                const bool hasWinner = !cmd.owner.empty() && cmd.emphasis > 0.0;
                const bool hasAvoid  = !cmd.avoidOwner.empty() && cmd.avoidEmphasis > 0.0;

                if (hasWinner && hasAvoid) {
                    const double avoidRatio = cmd.avoidEmphasis / (cmd.emphasis + cmd.avoidEmphasis);
                    const double goalRatio  = 1.0 - avoidRatio;

                    vL = (cmd.command.first  * goalRatio) + (cmd.avoidVL * avoidRatio);
                    vR = (cmd.command.second * goalRatio) + (cmd.avoidVR * avoidRatio);
                    winner = cmd.owner + "+avoid";

                } else if (hasWinner) {
                    vL = cmd.command.first;
                    vR = cmd.command.second;
                } else if (hasAvoid) {
                    vL = cmd.avoidVL;
                    vR = cmd.avoidVR;
                    winner = "avoid";
                    emphasis = cmd.avoidEmphasis;
                }
                setOutput("Motor", cmd);
            }
        }

        const std::string currentWinner = winner.empty() ? "none" : winner;
        const bool velChanged = std::abs(vL - prevVL) > 1e-4 || std::abs(vR - prevVR) > 1e-4;
        const bool winChanged = currentWinner != previousWinner;

        if (velChanged || winChanged) {
            sp.exec([&](RemoteAPIObjects& s) {
                try {
                    if (velChanged) {
                        s.sim().setJointTargetVelocity(leftMotor_,  vL / 0.0975);
                        s.sim().setJointTargetVelocity(rightMotor_, vR / 0.0975);
                    }
                    if (winChanged) {

                        s.sim().setStringSignal("active_behavior", currentWinner);
                    }
                } catch (const std::exception& e) {
                    std::cout << "[MotorArbitrator] error: " << e.what() << std::endl;
                }
            });
            prevVL = vL;
            prevVR = vR;
            previousWinner = currentWinner;
        }

        sp.arbitratorDone();
    }
    return BT::NodeStatus::IDLE;
}