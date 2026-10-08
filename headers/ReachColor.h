#ifndef REACH_COLOR_H
#define REACH_COLOR_H

#include "AttentionalBehavior.h"
#include "RemoteAPIObjects.h"
#include <string>

class ReachColor : public AttentionalBehavior {
private:
    RemoteAPIObjects* simObj_;
    std::string objectPath_;
    std::string color_;

public:
    ReachColor(const std::string& name, const BT::NodeConfig& config, RemoteAPIObjects* simObj);

    static BT::PortsList providedPorts();

    bool checkReleaser() override;
    void updatePerceptualSchema() override;
    std::pair<double, double> executeMotorSchema() override;
};

#endif // REACH_COLOR_H