 #include "AttentionalBehavior.h"

 AttentionalBehavior::AttentionalBehavior(const std::string& name, const BT::NodeConfig& config)
                                        : BT::ActionNodeBase(name, config) {}

BT::PortsList AttentionalBehavior::providedPorts() {
       return { BT::InputPort<double>("mu", 1.0, "Top-Down Regulation mu_b") };
}

 BT::NodeStatus AttentionalBehavior::tick() {
       updatePerceptualSchema();
        
       if (!checkInternalReleaser() || !checkExternalReleaser()) {
           emphasis_ = 0.0;
           return BT::NodeStatus::FAILURE;
       }

       if (lambda_ <= 1e-5) lambda_ = 1e-5;
       emphasis_ = mu_ / lambda_;
       std::cout << "[AttentionalBehavior] nuova emphasis di " << getUniqueID() << ": " << emphasis_ << std::endl;
       auto [vLeft, vRight] = executeMotorSchema();
       if(vLeft == 0.0 && vRight == 0.0){
              std::cout << getUniqueID() << " SUCCESS" << std::endl;
              MotorArbitrator::getInstance().proposeCommand(getUniqueID(), emphasis_, vLeft, vRight);
              return BT::NodeStatus::SUCCESS;
       }
       MotorArbitrator::getInstance().proposeCommand(getUniqueID(), emphasis_, vLeft, vRight);


       return BT::NodeStatus::RUNNING;
}

void AttentionalBehavior::halt(){
       emphasis_ = 0.0;
       resetStatus();
}

std::string AttentionalBehavior::getUniqueID() const {
    return name() + "_" + std::to_string(reinterpret_cast<uintptr_t>(this));
}