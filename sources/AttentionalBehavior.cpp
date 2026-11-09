#include "AttentionalBehavior.h"
 #include "SharedPerception.h"
 #include "MotorCommand.h"

AttentionalBehavior::AttentionalBehavior(const std::string& name, const BT::NodeConfig& config)
    : BT::ThreadedAction(name, config), sp(SharedPerception::get()) {}

BT::PortsList AttentionalBehavior::providedPorts() {
       return { 
        BT::BidirectionalPort<double>("mu", 1.0, "Top-Down Regulation mu_b"),
        BT::BidirectionalPort<MotorCommand>("Motor")
     };
}


BT::NodeStatus AttentionalBehavior::tick() {
    const uint64_t key = UID();
    sp.registerWorker();

    struct Guard {
        AttentionalBehavior* self; SharedPerception& sp; uint64_t key;
        ~Guard() { self->withdrawCommand(); sp.withdrawGoal(key); sp.unregisterWorker(); }
    } guard{this, sp, key};

    uint64_t seen = 0;
    nextActivation_ = 0.0;
    active_ = false;
    holdCounter_ = 0;
    
    while (!isHaltRequested()) {
        if (!sp.waitNextTick(seen, [this] { return isHaltRequested(); })) continue;
        const double now = sp.time();
        
        try {
            if (now >= nextActivation_) {
                if (evaluateCycle(now)) { 
        
                    withdrawCommand();
                    sp.workerDone();
                    return BT::NodeStatus::SUCCESS;
                }
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

void AttentionalBehavior::proposeCommand() {
    const std::string myID = getUniqueID() + "_" + std::to_string(UID());

    {
        std::lock_guard<std::mutex> lock(MotorCommand::lock()); 
        if (!config().blackboard->get("Motor", cmd)) return;

        if (exclusive()) {
            if (cmd.owner == myID || emphasis_ > cmd.emphasis) {
                //if (cmd.owner != myID) std::cout << "old owner: " << cmd.owner << " new owner: " << myID << " old e: " << cmd.emphasis << " new e: " << emphasis_ << std::endl;
                cmd.owner    = myID;
                cmd.emphasis = emphasis_;
                cmd.command  = {lastVL_, lastVR_};
            }
        } else {
            if (cmd.avoidOwner == myID || emphasis_ >= cmd.avoidEmphasis) {
                cmd.avoidOwner    = myID;
                cmd.avoidEmphasis = emphasis_;
                cmd.avoidVL       = lastVL_;
                cmd.avoidVR       = lastVR_;
            }
        }
        // std::cout << "NEW PROPOSE |  ID " << myID << " e:" << emphasis_ << std::endl;
        setOutput("Motor", cmd);
    }
}

void AttentionalBehavior::withdrawCommand() {
    const std::string myID = getUniqueID() + "_" + std::to_string(UID());

    std::lock_guard<std::mutex> lock(MotorCommand::lock());
    if (!getInput("Motor", cmd)) return;

    if (exclusive()) {
        if (cmd.owner == myID) {        
            cmd.owner    = "";
            cmd.emphasis = -1.0;
            cmd.command  = {0.0, 0.0};
        }
    } else {
        if (cmd.avoidOwner == myID) {
            cmd.avoidOwner    = "";
            cmd.avoidEmphasis = -1.0;
            cmd.avoidVL       = 0.0;
            cmd.avoidVR       = 0.0;
        }
    }
    
    setOutput("Motor", cmd);
}

void AttentionalBehavior::halt() {
    sp.wake();
    BT::ThreadedAction::halt();
    withdrawCommand();
    sp.withdrawGoal(UID());
    nextActivation_ = 0.0;
    active_ = false;
    lastVL_ = 0.0;
    lastVR_ = 0.0;
    accomplished_ = false;
}


std::string AttentionalBehavior::getUniqueID() const {
    std::string obj, color, target, place;
    getInput("Object", obj); getInput("Color", color);
    getInput("Object", target); getInput("Target", place);
    if (!exclusive()) return name() + "_" + (obj == "/Bill" ? obj : "");
    std::string id = name();
    if (obj == "/Bill") id += "_/Bill";
    else if (!target.empty()) id += "_" + target;
    if (!color.empty()) id += "_" + color;
    if (!place.empty()) id += "_" + place;
    return id;
}



bool AttentionalBehavior::evaluateCycle(double now) {
    updatePerceptualSchema();
    updateEmphasis();
    
    active_ = checkInternalReleaser() && checkExternalReleaser();

    if (active_) {
        std::tie(lastVL_, lastVR_) = executeMotorSchema();
        
        const bool stopped = (lastVL_ == 0.0 && lastVR_ == 0.0);
        if (stopped && name() != "avoid") {
            if (accomplishesTask()) taskAccomplished();
            return true;
        }
    } else {
        lastVL_ = 0.0;
        lastVR_ = 0.0;
    }
    
    nextActivation_ = now + 1.0 / std::max(emphasis_, 1e-3);
    
    return false; 
}

void AttentionalBehavior::updateEmphasis() {
    if (!getInput("mu", mu_)) mu_ = 1.0;
    if (lambda_ < 0.01) lambda_ = 0.01;
    emphasis_ = mu_ / lambda_;
}