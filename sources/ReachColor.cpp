#include "ReachColor.h"
#include "SharedPerception.h"
#include <iostream>
#include <cmath>
#include <algorithm>

static double hueOf(const std::string& c) { 
    if (c == "Red") return 0.0;   
    if (c == "Orange") return 30.0; 
    if (c == "Yellow") return 60.0; 
    if (c == "Green") return 120.0; 
    return -1.0; 
}

static int countHue(const std::vector<unsigned char>& img, double hueDeg, double tol = 12.0) { 
    int n = 0; 
    for (size_t i = 0; i + 2 < img.size(); i += 3) { 
        const double r = img[i] / 255.0, g = img[i+1] / 255.0, b = img[i+2] / 255.0; 
        const double mx = std::max({r, g, b}), mn = std::min({r, g, b}), d = mx - mn; 
        if (mx < 0.25 || d / mx < 0.5) continue; 
        double h = (mx == r) ? 60.0 * std::fmod((g - b) / d, 6.0)
                 : (mx == g) ? 60.0 * ((b - r) / d + 2.0)
                             : 60.0 * ((r - g) / d + 4.0); 
        if (h < 0) h += 360.0; 
        const double dh = std::min(std::abs(h - hueDeg), 360.0 - std::abs(h - hueDeg)); 
        if (dh < tol) ++n; 
    } 
    return n; 
}

ReachColor::ReachColor(const std::string& name, const BT::NodeConfig& config, RemoteAPIObjects* sim) 
    : AttentionalBehavior(name, config), sim_(sim) { 
    robotHandle_ = sim_->sim().getObjectHandle("/PioneerP3DX"); 
}

BT::PortsList ReachColor::providedPorts() { 
    BT::PortsList ports = { 
        BT::InputPort<std::string>("Color"), 
        BT::InputPort<std::string>("Object"),
    }; 
    auto basePorts = AttentionalBehavior::providedPorts(); 
    ports.insert(basePorts.begin(), basePorts.end()); 
    return ports; 
}

int64_t ReachColor::resolveTarget() { 
    if (target_ != previousTarget_) { 
        try { targetHandle_ = sim_->sim().getObjectHandle(target_); } catch(...) { targetHandle_ = -1; }
        previousTarget_ = target_; 
    } 
    return targetHandle_; 
}

double ReachColor::readSensors() { 
    distance_ = 999.0; 
    isTargetVisible_ = false;

    getInput("Object", target_);
    if (target_.empty()) { tracking_ = false; return distance_; }

    targetHandle_ = resolveTarget();
    if (targetHandle_ == -1) return distance_;

    try {
        if (target_ == "/Bill") {
            auto bp = sim_->sim().getObjectPosition(targetHandle_, -1);
            const auto& rp = SharedPerception::get().position();
            const double yaw = SharedPerception::get().orientation()[2];
            const double dx = bp[0] - rp[0], dy = bp[1] - rp[1];
            relX_ =  std::cos(yaw) * dx + std::sin(yaw) * dy;
            relY_ = -std::sin(yaw) * dx + std::cos(yaw) * dy;
            distance_ = std::hypot(relX_, relY_);
            isTargetVisible_ = true;
            return distance_;
        }
        
        const int64_t cam = SharedPerception::get().cameraHandle();
        static double cachedT = -1.0;
        static std::vector<unsigned char> cachedImg;
        const double now = SharedPerception::get().time();
        if (now != cachedT) {
            auto res = sim_->sim().getVisionSensorImg(cam);
            const auto& img = std::get<0>(res);
            cachedImg.assign(img.begin(), img.end());
            cachedT = now;
        }

        std::string color; 
        getInput("Color", color);
        const double hue = hueOf(color);
        const double tol = (hue == 30.0 || hue == 60.0) ? 8.0 : 12.0;
        const int px = countHue(cachedImg, hue, tol);

        auto pos = sim_->sim().getObjectPosition(targetHandle_, robotHandle_);
        relX_ = pos[0]; 
        relY_ = pos[1];
        distance_ = std::hypot(relX_, relY_);

        isTargetVisible_ = (px >= 30) || (tracking_ && distance_ < 1.0);
        tracking_ = isTargetVisible_;
        
    } catch (const std::exception& e) {
        tracking_ = false;
        std::cout << "EXCEPTION IN REACH COLOR: " << e.what() << std::endl;
    }

    return distance_; 
}

bool ReachColor::checkInternalReleaser() { 
    return isTargetVisible_; 
}

std::pair<double, double> ReachColor::executeMotorSchema() {
    if (distance_ <= 0.55) return {0.0, 0.0};

    double angle = std::atan2(relY_, relX_);
    double v_max = 0.40;
    
    if (distance_ < 1.0)
        v_max = 0.15 + (distance_ / 1.0) * 0.25;

    double v = 0.0;
    if (std::abs(angle) < (M_PI / 3.0))
        v = v_max * std::cos(angle);

    double w = std::clamp(angle * 1.8, -1.2, 1.2);
    double L = 0.331;
    
    double v_left  = v - (w * L) / 2.0;
    double v_right = v + (w * L) / 2.0;

    return {v_left, v_right};
}