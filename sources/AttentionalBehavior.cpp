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
              if(name() == "avoid" && !checkInternalReleaser()) return BT::NodeStatus::RUNNING;
              return BT::NodeStatus::FAILURE;
       }

       if (lambda_ <= 1e-5) lambda_ = 1e-5;
       emphasis_ = mu_ / lambda_;
       auto [vLeft, vRight] = executeMotorSchema();
       if(vLeft == 0.0 && vRight == 0.0){
              std::cout << getUniqueID() << " SUCCESS" << std::endl;
              MotorArbitrator::getInstance().proposeCommand(getUniqueID(), emphasis_, vLeft, vRight);
              if(name() == "avoid") return BT::NodeStatus::RUNNING;
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
       std::string target_, color, toReturn;
       getInput("ObjectPath",target_);
       if(!target_.empty() && target_ == "/Bill") 
              return name() + "_"+ target_ /*+ std::to_string(reinterpret_cast<uintptr_t>(this))*/;
       if(getInput("Color", color))
              return name() + "_"+ color /*+ std::to_string(reinterpret_cast<uintptr_t>(this))*/;
       return name() + "_" /*+ std::to_string(reinterpret_cast<uintptr_t>(this))*/;
}