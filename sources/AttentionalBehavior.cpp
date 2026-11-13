#include "AttentionalBehavior.h"
#include <iostream>

AttentionalBehavior::AttentionalBehavior(const std::string& name, const BT::NodeConfig& config)
    : BT::ThreadedAction(name, config), sp(SharedPerception::get()) {}

BT::PortsList AttentionalBehavior::providedPorts() {
    return {
        BT::BidirectionalPort<double>("mu", kDefaultMu, "Top-Down Regulation mu_b"),
        BT::BidirectionalPort<MotorCommand>("Motor")
    };
}

void AttentionalBehavior::resetState() {
    nextActivation_ = 0.0;
    lastVL_         = 0.0;
    lastVR_         = 0.0;
    active_         = false;
}

// void AttentionalBehavior::taskAccomplished() {
//     if (accomplished_) return;
//     accomplished_ = true;

//     double currentMu = kDefaultMu;
//     getInput("mu", currentMu);
//     setOutput("mu", currentMu + kMuStep);
// }

void AttentionalBehavior::updateEmphasis() {
    if (!getInput("mu", mu_)) mu_ = kDefaultMu;

    lambda_   = std::max(lambda_, kMinLambda);
    emphasis_ = mu_ / lambda_;
}

bool AttentionalBehavior::evaluateCycle(double now) {
    updatePerceptualSchema();
    updateEmphasis();

    active_ = checkInternalReleaser() && checkExternalReleaser();

    if (active_) {
        std::tie(lastVL_, lastVR_) = executeMotorSchema();

        if (isFinished()) {
            // if (accomplishesTask()) {
            //     taskAccomplished();
            // }
            return true;
        }
    } else {
        lastVL_ = 0.0;
        lastVR_ = 0.0;
    }

    nextActivation_ = now + 1.0 / std::max(emphasis_, kMinEmphasis);
    return false;
}

std::string AttentionalBehavior::getUniqueID() const {
    std::string obj, color, place;
    getInput("Object", obj);
    getInput("Color", color);
    getInput("Target", place);

    if (!exclusive()) 
        return name() + "_" + (obj == "/Bill" ? obj : "");
    
    std::string id = name();
    if (!obj.empty())   id += "_" + obj;
    if (!color.empty()) id += "_" + color;
    if (!place.empty()) id += "_" + place;
    return id;
}

std::string AttentionalBehavior::getProposalID() const {
    return getUniqueID() + "_" + std::to_string(UID());
}

void AttentionalBehavior::proposeCommand() {
    const std::string myID = getProposalID();
    
    std::lock_guard<std::mutex> lock(MotorCommand::lock());
    MotorCommand cmd;
    if (!getInput("Motor", cmd)) return;

    if (exclusive()) {
        if (cmd.owner == myID || emphasis_ > cmd.emphasis) {
            cmd.owner    = myID;
            cmd.emphasis = emphasis_;
            cmd.command  = {lastVL_, lastVR_};
            setOutput("Motor", cmd);
        }
    } else {
        if (cmd.avoidOwner == myID || emphasis_ >= cmd.avoidEmphasis) {
            cmd.avoidOwner    = myID;
            cmd.avoidEmphasis = emphasis_;
            cmd.avoidVL       = lastVL_;
            cmd.avoidVR       = lastVR_;
            setOutput("Motor", cmd);
        }
    }
}

void AttentionalBehavior::withdrawCommand() {
    const std::string myID = getProposalID();

    std::lock_guard<std::mutex> lock(MotorCommand::lock());
    MotorCommand cmd;
    if (!getInput("Motor", cmd)) return;

    if (exclusive()) {
        if (cmd.owner == myID) {
            cmd.owner    = "";
            cmd.emphasis = -1.0;
            cmd.command  = {0.0, 0.0};
            setOutput("Motor", cmd);
        }
    } else {
        if (cmd.avoidOwner == myID) {
            cmd.avoidOwner    = "";
            cmd.avoidEmphasis = -1.0;
            cmd.avoidVL       = 0.0;
            cmd.avoidVR       = 0.0;
            setOutput("Motor", cmd);
        }
    }
}

BT::NodeStatus AttentionalBehavior::tick() {
    const uint64_t key = UID();
    sp.registerWorker();

    struct WorkerScopeGuard {
        AttentionalBehavior* self;
        SharedPerception& sp;
        uint64_t key;
        ~WorkerScopeGuard() {
            self->withdrawCommand();
            sp.withdrawGoal(key);
            sp.unregisterWorker();
        }
    } guard{this, sp, key};

    uint64_t seen = 0;
    resetState();

    while (!isHaltRequested()) {
        if (!sp.waitNextTick(seen, [this] { return isHaltRequested(); }))
            continue;
        
        const double now = sp.time();

        try {
            if (now >= nextActivation_ && evaluateCycle(now)) {
                withdrawCommand();
                sp.workerDone();
                return BT::NodeStatus::SUCCESS;
            }

            if (active_) {
                proposeCommand();
                afterPropose();
            } else {
                withdrawCommand();
                sp.withdrawGoal(key);
            }
        } catch (const std::exception& e) {
            std::cout << "[" << name() << "] " << e.what() << '\n';
        }

        sp.workerDone();
    }

    return BT::NodeStatus::IDLE;
}

void AttentionalBehavior::halt() {
    sp.wake();
    BT::ThreadedAction::halt();
    withdrawCommand();
    sp.withdrawGoal(UID());
    resetState();
    //accomplished_ = false;
}