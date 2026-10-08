#ifndef CONDITIONS_H
#define CONDITIONS_H

#include <behaviortree_cpp/condition_node.h>
#include "RemoteAPIObjects.h"
#include <string>

class IsColorReached : public BT::ConditionNode {
private:
    RemoteAPIObjects* sim_;

public:
    IsColorReached(const std::string& name, const BT::NodeConfig& config, RemoteAPIObjects* sim_);

    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;
};


class IsObjectInKnapsack : public BT::ConditionNode {
private:
    RemoteAPIObjects* sim_;

public:
    IsObjectInKnapsack(const std::string& name, const BT::NodeConfig& config, RemoteAPIObjects* sim_);

    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;
};

#endif // CONDITIONS_H