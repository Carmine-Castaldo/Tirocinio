#ifndef ATTENTIONAL_BEHAVIOR_H
#define ATTENTIONAL_BEHAVIOR_H

#include <behaviortree_cpp/action_node.h>
#include "MotorArbitrator.h"
#include <cmath>
#include <string>
#include <utility>



class AttentionalBehavior : public BT::ActionNodeBase {
protected:
    double mu_ = 1.0;
    double lambda_ = 1.0;
    double emphasis_ = 0.0;

public:
    AttentionalBehavior(const std::string& name, const BT::NodeConfig& config)
        : BT::ActionNodeBase(name, config) {}

    static BT::PortsList providedPorts() {
        return { BT::InputPort<double>("mu", 1.0, "Top-Down Regulation mu_b") };
    }

    virtual bool checkReleaser() = 0;
    virtual void updatePerceptualSchema() = 0;
    virtual std::pair<double, double> executeMotorSchema() = 0;

    BT::NodeStatus tick() override {
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

    void halt() override {
        emphasis_ = 0.0;
        setStatus(BT::NodeStatus::IDLE);
    }

    double getEmphasis() const { return emphasis_; }
    double getLambda() const { return lambda_; }
    double getMu() const { return mu_; }
};

#endif // ATTENTIONAL_BEHAVIOR_H