#ifndef PLACE_H
#define PLACE_H

#include "AttentionalBehavior.h"
#include "RemoteAPIObjects.h"
#include <string>
#include <vector>

class Place : public AttentionalBehavior {
private:
    RemoteAPIObjects* sim_;
    std::string objectPath_, target_, previousPath_;
    std::vector<int> knapsack;
    int64_t targetHandle_ = -1;
    double distance = 999.0;

public:
    Place(const std::string& name, const BT::NodeConfig& config, RemoteAPIObjects* sim_);

    static BT::PortsList providedPorts();

    bool checkInternalReleaser() override;
    bool checkExternalReleaser() override;
    void updatePerceptualSchema() override;
    std::pair<double, double> executeMotorSchema() override;
    virtual bool accomplishesTask() const { return true; }


};

#endif // PLACE_H
