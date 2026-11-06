 #include "AttentionalBehavior.h"
 #include "SharedPerception.h"

 AttentionalBehavior::AttentionalBehavior(const std::string& name, const BT::NodeConfig& config)
                                        : BT::ThreadedAction(name, config) {}

BT::PortsList AttentionalBehavior::providedPorts() {
       return { BT::BidirectionalPort<double>("mu", 1.0, "Top-Down Regulation mu_b") };
}

BT::NodeStatus AttentionalBehavior::tick() {
    auto& sp  = SharedPerception::get();
    auto& arb = MotorArbitrator::getInstance();
    const uint64_t key = UID();
    sp.registerWorker();
    struct Guard { SharedPerception& sp; MotorArbitrator& arb; uint64_t key;
        ~Guard(){ arb.withdraw(key); sp.withdrawGoal(key); sp.unregisterWorker(); } } guard{sp, arb, key};

    uint64_t seen = 0;
    nextActivation_ = 0.0; active_ = false;
    while (!isHaltRequested()) {
        if (!sp.waitNextTick(seen, [this]{ return isHaltRequested(); })) continue;
        const double now = sp.time();
        try {
            if (now >= nextActivation_) {
                updatePerceptualSchema();
                if (!getInput("mu", mu_)) mu_ = 1.0;
                if (lambda_ < 0.01) lambda_ = 0.01;
                emphasis_ = mu_ / lambda_;

                active_ = checkInternalReleaser() && checkExternalReleaser();
                if (active_) {
                    std::tie(lastVL_, lastVR_) = executeMotorSchema();
                    const bool stopped = (lastVL_ == 0.0 && lastVR_ == 0.0);
                    if (stopped && name() != "avoid") {
                        if (accomplishesTask()) taskAccomplished();
                        sp.workerDone();
                        return BT::NodeStatus::SUCCESS;
                    }
                } else { lastVL_ = lastVR_ = 0.0; }
                nextActivation_ = now + 1.0 / std::max(emphasis_, 1e-3);
            }
            if (active_) { arb.propose(key, getUniqueID(), emphasis_, lastVL_, lastVR_, exclusive()); afterPropose(); }
            else         { arb.withdraw(key); sp.withdrawGoal(key); }
        } catch (const std::exception& e) { std::cout << "[" << name() << "] " << e.what() << std::endl; }
        sp.workerDone();
    }
    return BT::NodeStatus::IDLE;
}

void AttentionalBehavior::halt() {
    SharedPerception::get().wake();
    BT::ThreadedAction::halt();
    MotorArbitrator::getInstance().withdraw(UID()); 
    SharedPerception::get().withdrawGoal(UID());
    nextActivation_ = 0.0; active_ = false; lastVL_ = lastVR_ = 0.0; accomplished_ = false;
}

std::string AttentionalBehavior::getUniqueID() const {
    std::string obj, color, target, place;
    getInput("Object", obj); getInput("Color", color);
    getInput("Object", target);  getInput("Target", place);
    if (!exclusive()) return name() + "_" + (obj == "/Bill" ? obj : "");
    std::string id = name();
    if (obj == "/Bill") id += "_/Bill";
    else if (!target.empty()) id += "_" + target;
    if (!color.empty()) id += "_" + color;
    if (!place.empty()) id += "_" + place;
    return id;
}