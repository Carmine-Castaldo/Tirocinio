#include "AttentionalBehavior.h"

AttentionalBehavior::AttentionalBehavior(const std::string& name, const BT::NodeConfig& config) 
    : BT::ActionNodeBase(name, config) {}

BT::PortsList AttentionalBehavior::providedPorts() {
    return {
        BT::OutputPort<double>("stimuli", "sigma"),
        BT::InputPort<double>("emphasis"),
        BT::InputPort<double>("period"),
        BT::InputPort<double>("lambda"), 
        BT::InputPort<double>("mu"),
        BT::InputPort<double>("external")
    };
}
BT::NodeStatus AttentionalBehavior::tick() {
    const double now = SharedPerception::get().time();

    getInput("emphasis", emphasis_); 
    getInput("period",   period_);
    if (emphasis_ <= 0.0) emphasis_ = 1e-3;
    if (period_   <= 0.0) period_   = 1.0;

    BT::NodeStatus currentStatus = BT::NodeStatus::RUNNING;

    if (now >= lastActivation_ + period_) { 
        lastActivation_ = now;
        setOutput("stimuli", readSensors());
        double ext = 1.0; getInput("external", ext);
        active_ = checkInternalReleaser() && (ext > 0.5);
        if (active_) {
            std::tie(lastVL_, lastVR_) = executeMotorSchema();
            const bool stopped = (lastVL_ == 0.0 && lastVR_ == 0.0);
            currentStatus = (stopped && name() != "avoid") ? BT::NodeStatus::SUCCESS
                                                           : BT::NodeStatus::RUNNING;
        } else { lastVL_ = lastVR_ = 0.0; }
    }
    if (active_) {
        MotorArbitrator::getInstance().proposeCommand(getUniqueID(), emphasis_, lastVL_, lastVR_, exclusive());
        afterPropose();
    }
    return currentStatus;
}

void AttentionalBehavior::halt() {
    lastActivation_ = -1e9; 
    lastVL_ = 0.0;
    lastVR_ = 0.0;
    active_ = false;
    resetStatus();
}

    std::string AttentionalBehavior::getUniqueID() const { 
        std::string obj, color, place; 
        getInput("Object", obj); 
        getInput("Color", color); 
        getInput("Target", place); 
        
        if (!exclusive()) return name() + " " + (obj == "/Bill" ? obj : "");
        
        std::string id = name();
        if (obj == "/Bill") id += "_/Bill"; 
        else if (!obj.empty()) id += "_" + obj;
        if (!color.empty()) id += "_" + color; 
        if (!place.empty()) id += "_" + place; 
        
        return id; 
    }