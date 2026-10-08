#include "Conditions.h"
#include <cmath>
#include <iostream>

IsColorReached::IsColorReached(const std::string& name, const BT::NodeConfig& config, RemoteAPIObjects* sim)
    : BT::ConditionNode(name, config), sim_(sim) {}

BT::PortsList IsColorReached::providedPorts()
{
    return {
        BT::InputPort<std::string>("Object", "Object Path in 'CoppeliaSim'"),
        BT::InputPort<std::string>("Color")
    };
}

BT::NodeStatus IsColorReached::tick()
{
    std::string objectPath;
    if (!getInput("Object", objectPath)) 
        return BT::NodeStatus::FAILURE;

    try {
        if (sim_->getDistanceTo(objectPath) <= 0.45)
            return BT::NodeStatus::SUCCESS;
        
    } catch (const std::exception& e) {
        e.what();
        return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::FAILURE;
}


IsObjectInKnapsack::IsObjectInKnapsack(const std::string& name, const BT::NodeConfig& config, RemoteAPIObjects* sim)
    : BT::ConditionNode(name, config), sim_(sim)
{
}

BT::PortsList IsObjectInKnapsack::providedPorts()
{
    return {
        BT::InputPort<std::string>("Object", "Object Path in 'CoppeliaSim'"),
        BT::InputPort<std::vector<int>>("Knapsack", "Knapsack max 4 Object")
    };
}

BT::NodeStatus IsObjectInKnapsack::tick()
{
    std::string objPath;
    std::vector<int> knapsackVec;

    if (!getInput("Object", objPath)) {
        return BT::NodeStatus::FAILURE;
    }

    if (!getInput("Knapsack", knapsackVec)) {
        return BT::NodeStatus::FAILURE;
    }

    try {
        int targetHandle = static_cast<int>(sim_->sim().getObjectHandle(objPath));
        if (std::find(knapsackVec.begin(), knapsackVec.end(), targetHandle) != knapsackVec.end())
            return BT::NodeStatus::SUCCESS;
        
    } catch (...) {
        return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::FAILURE;
}