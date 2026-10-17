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
    int64_t drawHandle_ = -1;
    bool active_ = false, accomplished_= false;
    static constexpr double kMuStep = 1.0;   
    double nextActivation_ = 0.0;
    double lastVL_ = 0.0, lastVR_ = 0.0;
    BT::NodeStatus lastStatus_ = BT::NodeStatus::RUNNING;
    virtual void afterPropose() {};
    static double saturatingLambda(double s, double rmin, double rmax,
                               double lmin = 0.01, double lmax = 1.0) {
        if (s <= rmin) return lmin;
        if (s >= rmax) return lmax;
        return lmin + (lmax - lmin) * (s - rmin) / (rmax - rmin);
    }
    virtual bool accomplishesTask() const { return false; }
    
    void taskAccomplished() {
        if (accomplished_) return;
        accomplished_ = true;
        double cur = 1.0;
        getInput("mu", cur); 
        setOutput("mu", cur + kMuStep);
    }
    

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
    virtual bool exclusive() const {return true;}
};

#endif // ATTENTIONAL_BEHAVIOR_H