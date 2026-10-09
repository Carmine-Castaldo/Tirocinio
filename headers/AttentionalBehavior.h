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
    AttentionalBehavior(const std::string& name, const BT::NodeConfig& config);

    static BT::PortsList providedPorts();

    virtual bool checkInternalReleaser() = 0;
    virtual bool checkExternalReleaser() = 0;
    virtual void updatePerceptualSchema() = 0;
    virtual std::pair<double, double> executeMotorSchema() = 0;
    std::string getUniqueID() const;

    BT::NodeStatus tick() override;
    void halt() override;

    double getEmphasis() const { return emphasis_; }
    double getLambda() const { return lambda_; }
    double getMu() const { return mu_; }
    void setMu(double mu) { mu_ = mu;}
};

#endif // ATTENTIONAL_BEHAVIOR_H