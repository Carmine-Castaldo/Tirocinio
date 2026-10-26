#include "Wander.h"
#include "SharedPerception.h"
#include <algorithm>
#include <cmath>

Wander::Wander(const std::string& name, const BT::NodeConfig& config, RemoteAPIObjects* sim)
    : AttentionalBehavior(name, config), sim_(sim), 
      gen_(std::random_device{}()), distDir_(-0.6, 0.6) {}

BT::PortsList Wander::providedPorts() {
    return AttentionalBehavior::providedPorts(); 
}
double Wander::readSensors() {
    return 1.0;
}

bool Wander::checkInternalReleaser() {
    return true; 
}
std::pair<double, double> Wander::executeMotorSchema() {
    const double now = SharedPerception::get().time();
    
    if (now > nextChangeTime_) {
        currentW_ += distDir_(gen_);
        currentW_ = std::clamp(currentW_, -0.6, 0.6); 
        nextChangeTime_ = now + 2.0;
    }

    double v = 0.25;
    double w = currentW_;

    double L = 0.331; 
    double v_left  = v - w * L / 2.0;
    double v_right = v + w * L / 2.0;

    return {v_left, v_right};
}