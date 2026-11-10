#ifndef PLACE_H
#define PLACE_H

#include "AttentionalBehavior.h"
#include "RemoteAPIObjects.h"
#include <string>
#include <vector>

class Place : public AttentionalBehavior {
private:
    static constexpr double kActionSeconds = 2.5;
    static constexpr double kMaxDistance   = 0.80;

    RemoteAPIObjects* sim_;
    std::string objectPath_;
    std::string target_;
    std::string previousPath_;
    std::vector<int> knapsack_;
    int64_t targetHandle_ = -1;
    double distance_      = 999.0;

    double startTime_ = -1.0;
    bool finished_    = false;

    void completeAction();

protected:
    bool isFinished() const override { return finished_; }

    void resetState() {
        AttentionalBehavior::resetState();
        startTime_ = -1.0;
        finished_  = false;
    }

    void afterPropose() override;

public:
    Place(const std::string& name, const BT::NodeConfig& config, RemoteAPIObjects* sim);

    static BT::PortsList providedPorts();

    bool checkInternalReleaser() override;
    bool checkExternalReleaser() override;
    void updatePerceptualSchema() override;
    std::pair<double, double> executeMotorSchema() override;
};

#endif // PLACE_H