#pragma once
#include <behaviortree_cpp/action_node.h>
#include "SharedPerception.h"
#include "MotorArbitrator.h"

class AttentionalBehavior : public BT::ActionNodeBase {
protected:
    double emphasis_ = 1.0;
    double period_ = 1.0;
    double nextActivation_ = 0.0, lastActivation_ = -1e9;
    
    double lastVL_ = 0.0;
    double lastVR_ = 0.0;
    
    bool active_ = false;

public:
    AttentionalBehavior(const std::string& name, const BT::NodeConfig& config);

    static BT::PortsList providedPorts();

    virtual double readSensors() = 0; 
    virtual std::pair<double, double> executeMotorSchema() = 0;
    
    virtual bool checkInternalReleaser() { return true; } 
    virtual bool exclusive() const { return true; }
    virtual void afterPropose() {}
    BT::NodeStatus tick() override;
    void halt() override;
    
    std::string getUniqueID() const;
};