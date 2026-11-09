#ifndef ATTENTIONAL_BEHAVIOR_H
#define ATTENTIONAL_BEHAVIOR_H

#include <behaviortree_cpp/action_node.h>

#include "MotorCommand.h"
#include "SharedPerception.h"

class AttentionalBehavior : public BT::ThreadedAction {
protected:
    int holdCounter_ = 0;
    int64_t drawHandle_ = -1;
    bool active_ = false, accomplished_ = false;
    double mu_ = 1.0, lambda_ = 1.0, emphasis_ = 0.0;
    double nextActivation_ = 0.0, lastVL_ = 0.0, lastVR_ = 0.0;
    static constexpr double kMuStep = 1.0;   
    MotorCommand cmd;
    SharedPerception &sp;
    BT::NodeStatus lastStatus_ = BT::NodeStatus::RUNNING;

    virtual void afterPropose() {}

    static double saturatingLambda(double s, double rmin, double rmax,
                                   double lmin = 0.01, double lmax = 1.0) {
        if (s <= rmin) return lmin;
        if (s >= rmax) return lmax;
        return lmin + (lmax - lmin) * (s - rmin) / (rmax - rmin);
    }

    virtual bool accomplishesTask() const { return false; }
    virtual bool exclusive() const { return true; }

    void proposeCommand();
    void withdrawCommand();
    bool evaluateCycle(double now);
    void updateEmphasis();

    void taskAccomplished() {
        if (accomplished_) return;
        accomplished_ = true;
        double cur = 1.0;
        getInput("mu", cur); 
        setOutput("mu", cur + kMuStep);
    }

public:
    AttentionalBehavior(const std::string& name, const BT::NodeConfig& config);
    virtual ~AttentionalBehavior() override = default;

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