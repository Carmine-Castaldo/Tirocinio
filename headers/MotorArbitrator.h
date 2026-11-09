#ifndef MOTORARBITRATOR_H
#define MOTORARBITRATOR_H

#include <behaviortree_cpp/action_node.h>
#include "AttentionalBehavior.h"
#include "MotorCommand.h"

class MotorArbitrator : public BT::ThreadedAction {
private:
    int64_t leftMotor_  = -1;
    int64_t rightMotor_ = -1;
    MotorCommand cmd;
public:
    
    MotorArbitrator(const std::string& name, const BT::NodeConfig& config);
    
    static BT::PortsList providedPorts();
    
    BT::NodeStatus tick() override;
    void halt() override;
};

#endif // MOTORARBITRATOR_H