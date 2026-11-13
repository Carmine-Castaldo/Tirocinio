#ifndef ATTENTIONAL_BEHAVIOR_H
#define ATTENTIONAL_BEHAVIOR_H

#include <behaviortree_cpp/action_node.h>
#include <algorithm>
#include <string>
#include <utility>

#include "MotorCommand.h"
#include "SharedPerception.h"

class AttentionalBehavior : public BT::ActionNodeBase {
protected:
    static constexpr double kDefaultMu   = 1.0;
    static constexpr double kMinLambda   = 0.01;
    static constexpr double kMinEmphasis = 1e-3;

    SharedPerception& sp;

    bool active_           = false;
    double mu_             = kDefaultMu;
    double lambda_         = 1.0;
    double emphasis_       = 0.0;
    double nextActivation_ = 0.0;
    double lastVL_         = 0.0;
    double lastVR_         = 0.0;

    virtual void afterPropose() {}
    
    static double saturatingLambda(double s, double rmin, double rmax,
                                   double lmin = kMinLambda, double lmax = 1.0) {
        if (s <= rmin) return lmin;
        if (s >= rmax) return lmax;
        return lmin + (lmax - lmin) * (s - rmin) / (rmax - rmin);
    }

    virtual void resetState();
    virtual bool isFinished() const {
        return name() != "avoid" && lastVL_ == 0.0 && lastVR_ == 0.0;
    }

    void updateEmphasis();
    bool evaluateCycle(double now);
    void taskAccomplished();

    std::string getProposalID() const;
    void proposeCommand(bool updated);
    void withdrawCommand();

public:
    AttentionalBehavior(const std::string& name, const BT::NodeConfig& config);
    ~AttentionalBehavior() override = default;

    static BT::PortsList providedPorts();

    virtual bool checkInternalReleaser() = 0;
    virtual bool checkExternalReleaser() = 0;
    virtual void updatePerceptualSchema() = 0;
    virtual std::pair<double, double> executeMotorSchema() = 0;

    std::string getUniqueID() const;

    BT::NodeStatus tick() override;
    void halt() override;
};

#endif // ATTENTIONAL_BEHAVIOR_H