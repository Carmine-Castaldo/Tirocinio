#include "MotorArbitrator.h"
#include "SharedPerception.h"
#include <cmath>
#include <iostream>
#include <mutex>
#include <thread>

MotorArbitrator::MotorArbitrator(const std::string& name, const BT::NodeConfig& config)
    : BT::ThreadedAction(name, config) {
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
    //SharedPerception::get().wake();
    BT::ThreadedAction::halt();
}

MotorArbitrator::ArbitratedOutput MotorArbitrator::resolveCommand() {
    ArbitratedOutput out;

    std::lock_guard<std::mutex> lock(MotorCommand::lock());
    MotorCommand cmd;
    if (!getInput("Motor", cmd)) return out;
    
    MotorCommand::arbReadsTotal.fetch_add(1, std::memory_order_relaxed);
    const bool hasWinner = !cmd.owner.empty() && cmd.emphasis > 0.0;

     if (hasWinner) {
        out.vL       = cmd.command.first;
        out.vR       = cmd.command.second;
        out.winner   = cmd.owner;
        out.emphasis = cmd.emphasis;
    } 
    return out;
}

void MotorArbitrator::applyToSimulator(const ArbitratedOutput& out,
                                       std::string& previousWinner,
                                       double& prevVL,
                                       double& prevVR) {
    const bool stopping   = out.vL == 0.0 && out.vR == 0.0 && (prevVL != 0.0 || prevVR != 0.0);
    const bool velChanged = stopping ||
                            std::abs(out.vL - prevVL) > kVelEpsilon ||
                            std::abs(out.vR - prevVR) > kVelEpsilon;
    const bool winChanged = out.winner != previousWinner;

    if (!velChanged && !winChanged) return;
    
    const bool applied = SharedPerception::get().tryExec([&](RemoteAPIObjects& s) {
        try {
            if (velChanged) {
                s.sim().setJointTargetVelocity(leftMotor_,  out.vL / kWheelRadius);
                s.sim().setJointTargetVelocity(rightMotor_, out.vR / kWheelRadius);
            }
            if (winChanged)
                s.sim().setStringSignal("active_behavior", out.winner);
        } catch (const std::exception& e) {
            std::cout << "[MotorArbitrator] error: " << e.what() << '\n';
        }
    });

    if (applied) {
        prevVL         = out.vL;
        prevVR         = out.vR;
        previousWinner = out.winner;
    }
}


BT::NodeStatus MotorArbitrator::tick() {
    std::string previousWinner = "none";
    double prevVL = 1e9;
    double prevVR = 1e9;

    while (!isHaltRequested()) {
        //std::this_thread::sleep_for(std::chrono::milliseconds(5));
        const auto nextWake = std::chrono::steady_clock::now() + std::chrono::milliseconds(2);
        
        const ArbitratedOutput out = resolveCommand();
        applyToSimulator(out, previousWinner, prevVL, prevVR);
        if (isHaltRequested()) break;
        std::this_thread::sleep_until(nextWake);
    }

    return BT::NodeStatus::IDLE;
}