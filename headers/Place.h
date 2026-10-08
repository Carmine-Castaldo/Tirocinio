#ifndef PLACE_H
#define PLACE_H

#include "AttentionalBehavior.h"
#include "RemoteAPIObjects.h"
#include <string>

class Place : public AttentionalBehavior {
private:
    RemoteAPIObjects* simObj_;
    std::string objectPath_;
    std::string target_;
    bool isPlaced_ = false;

public:
    Place(const std::string& name, const BT::NodeConfig& config, RemoteAPIObjects* simObj);

    static BT::PortsList providedPorts();

    bool checkReleaser() override;
    void updatePerceptualSchema() override;
    std::pair<double, double> executeMotorSchema() override;
};

#endif // PLACE_H
