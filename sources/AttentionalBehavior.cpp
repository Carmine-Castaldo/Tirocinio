 #include "AttentionalBehavior.h"
 #include "SharedPerception.h"

 AttentionalBehavior::AttentionalBehavior(const std::string& name, const BT::NodeConfig& config)
                                        : BT::ActionNodeBase(name, config) {}

BT::PortsList AttentionalBehavior::providedPorts() {
       return { BT::BidirectionalPort<double>("mu", 1.0, "Top-Down Regulation mu_b") };
}
BT::NodeStatus AttentionalBehavior::tick() {
    const double now = SharedPerception::get().time();

    if (now >= nextActivation_) {
        updatePerceptualSchema();
        if(!getInput("mu", mu_)) mu_ = 1.0;
        if (lambda_ < 0.01) lambda_ = 0.01;
        emphasis_ = mu_ / lambda_;

        const bool internalOk = checkInternalReleaser();
        const bool externalOk = internalOk && checkExternalReleaser();
        active_ = internalOk && externalOk;

        if (active_) {
            std::tie(lastVL_, lastVR_) = executeMotorSchema();
            const bool stopped = (lastVL_ == 0.0 && lastVR_ == 0.0);
            
            lastStatus_ = (stopped && name() != "avoid") ? BT::NodeStatus::SUCCESS 
                                                        : BT::NodeStatus::RUNNING;
        } else {
            lastVL_ = 0.0;
            lastVR_ = 0.0;
            lastStatus_ = BT::NodeStatus::RUNNING;
        }

       const double e = std::max(emphasis_, 1e-3); // altrimenti, a volte, period va a inf
       const double period = 1.0 / e;
       nextActivation_ = now + period;

       std::cout << "[CLOCK] t=" << now << " " << getUniqueID()
          << " lambda=" << lambda_ << " mu=" << mu_ << " e=" << emphasis_
          << " p=" << period << std::endl;
    }
    
    if (lastStatus_ == BT::NodeStatus::SUCCESS) {
        if (!accomplished_) {
              accomplished_ = true;
              if (accomplishesTask()) taskAccomplished();
       }
    } else accomplished_ = false;
       
    if (active_) {
        MotorArbitrator::getInstance().proposeCommand(getUniqueID(), emphasis_, lastVL_, lastVR_, exclusive());
        afterPropose();
    }
    return lastStatus_;
}

void AttentionalBehavior::halt(){
       accomplished_ = false;
       resetStatus();
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