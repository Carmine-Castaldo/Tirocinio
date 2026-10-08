 #include "AttentionalBehavior.h"

 AttentionalBehavior::AttentionalBehavior(const std::string& name, const BT::NodeConfig& config)
                                        : BT::ActionNodeBase(name, config) {}

 BT::NodeStatus AttentionalBehavior::tick()  {
        getInput("mu", mu_);

        if (!checkReleaser()) {
            emphasis_ = 0.0;
            return BT::NodeStatus::FAILURE;
        }

        updatePerceptualSchema();

        if (lambda_ <= 1e-5) lambda_ = 1e-5;
        emphasis_ = mu_ / lambda_;

        auto [vLeft, vRight] = executeMotorSchema();

        MotorArbitrator::getInstance().proposeCommand(name(), emphasis_, vLeft, vRight);

        return BT::NodeStatus::RUNNING;
 }

 void AttentionalBehavior::halt(){
        emphasis_ = 0.0;
        setStatus(BT::NodeStatus::IDLE);
 }