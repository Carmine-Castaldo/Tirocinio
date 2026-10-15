#include "Wander.h"
#include "SharedPerception.h"
#include "TrialSetup.h"

Wander::Wander(const std::string& name, const BT::NodeConfig& config, RemoteAPIObjects* sim)
        : AttentionalBehavior(name, config), sim_(sim) {
    rng_.seed(trialSeed());
    targetX_ = dist_(rng_);
    targetY_ = dist_(rng_);
    lambda_ = 1.0;
    robotHandle_ = sim_->sim().getObjectHandle("/PioneerP3DX");

}

BT::PortsList Wander::providedPorts() {
    return AttentionalBehavior::providedPorts();
}

void Wander::updatePerceptualSchema() {
    const auto& robotPos = SharedPerception::get().position();
    const auto& ori = SharedPerception::get().orientation();
    
    double dx = targetX_ - robotPos[0];
    double dy = targetY_ - robotPos[1];
    current_dist_ = std::sqrt(dx * dx + dy * dy);
    double targetAngle = std::atan2(dy, dx);
    angleError_ = std::atan2(std::sin(targetAngle - ori[2]), std::cos(targetAngle - ori[2]));
  const double t = SharedPerception::get().time();
    if (t >= retargetAt_ || current_dist_ < 0.3) {
        targetX_ = dist_(rng_);  targetY_ = dist_(rng_);
        retargetAt_ = t + 15.0;
    }
    lambda_ = 1.0;
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
