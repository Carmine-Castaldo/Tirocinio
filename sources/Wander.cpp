#include "Wander.h"

Wander::Wander(const std::string& name, const BT::NodeConfig& config, RemoteAPIObjects* sim)
        : AttentionalBehavior(name, config), sim_(sim) {
    targetX_ = dist_(rng_);
    targetY_ = dist_(rng_);
    lambda_ = 15.0;
    robotHandle_ = sim_->sim().getObjectHandle("/PioneerP3DX");
    setMu(0.2);
}

BT::PortsList Wander::providedPorts() {
    return AttentionalBehavior::providedPorts();
}

void Wander::updatePerceptualSchema() {
    auto robotPos = sim_->getRobotPosition();
    double dx = targetX_ - robotPos[0];
    double dy = targetY_ - robotPos[1];
    current_dist_ = std::sqrt(dx * dx + dy * dy);
    std::vector<double> ori = sim_->sim().getObjectOrientation(robotHandle_, -1);
    double targetAngle = std::atan2(dy, dx);
    angleError_ = std::atan2(std::sin(targetAngle - ori[2]), std::cos(targetAngle - ori[2]));
    if (lambda_ <= 1.0 || current_dist_ < 0.3) {
        targetX_ = dist_(rng_);
        targetY_ = dist_(rng_);
        lambda_ = 15.0; 
    } else {
        lambda_ -= 1.0; 
    }
}

bool Wander::checkInternalReleaser() {
    return true;
}

bool Wander::checkExternalReleaser() {
    return true;    
}

std::pair<double, double> Wander::executeMotorSchema() {

    double v = 0.2 * std::max(0.0, std::cos(angleError_));
    double w = std::clamp(angleError_ * 1.0, -0.8, 0.8);

    double v_left  = v - w * 0.331 / 2.0;
    double v_right = v + w * 0.331 / 2.0;
    return {v_left, v_right};
}
