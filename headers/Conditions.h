#ifndef CONDITIONS_H
#define CONDITIONS_H

#include <behaviortree_cpp/condition_node.h>
#include "RemoteAPIObjects.h"
#include <string>

class IsColorReached : public BT::SimpleConditionNode {
public:
    IsColorReached(const std::string& name, const BT::NodeConfig& config);
    static BT::PortsList providedPorts();
};

class IsObjectInKnapsack : public BT::SimpleConditionNode {
public:
    IsObjectInKnapsack(const std::string& name, const BT::NodeConfig& config);
    static BT::PortsList providedPorts();
};

#endif // CONDITIONS_H