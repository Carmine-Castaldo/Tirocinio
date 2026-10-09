#include "AvoidObstacles.h"

AvoidObstacles::AvoidObstacles(const std::string& name, const BT::NodeConfig& config, RemoteAPIObjects* sim)
                : AttentionalBehavior(name, config), sim_(sim){
                    int64_t leftSonar = sim_->sim().getObjectHandle("/PioneerP3DX/ultrasonicSensor[3]");
                    int64_t frontSonar = sim_->sim().getObjectHandle("/PioneerP3DX/ultrasonicSensor[4]");
                    int64_t rightSonar = sim_->sim().getObjectHandle("/PioneerP3DX/ultrasonicSensor[5]");
                    sonarHandles_ = {leftSonar, frontSonar, rightSonar};
                }

BT::PortsList AvoidObstacles::providedPorts(){
    BT::PortsList ports = {
        BT::InputPort<std::string>("ObjectPath")
    };
    auto basePorts = AttentionalBehavior::providedPorts();
    ports.insert(basePorts.begin(), basePorts.end());
    return ports;
}

bool isTargetOrChild(RemoteAPIObjects* sim, int64_t detected, int64_t target) {
    if (detected == -1 || target == -1) return false;
    int64_t current = detected;
    while (current != -1) {
        if (current == target) return true;
        try {
            current = sim->sim().getObjectParent(current);
        } catch (...) {
            break;
        }
    }
    return false;
}


double AvoidObstacles::checkSonar(int64_t sonarHandle) {
    auto readSonar = sim_->sim().readProximitySensor(sonarHandle);
    if (std::get<0>(readSonar) == 1) {
        int64_t detectedHandle = std::get<3>(readSonar);
        if (!isTargetOrChild(sim_, detectedHandle, targetHandle_))
            return std::get<1>(readSonar);
    }
    
    return -1.0;
}

void AvoidObstacles::updatePerceptualSchema() {
    std::string path;
    targetHandle_ = -1;
    if (getInput("ObjectPath", path) && !path.empty()) {
        try { targetHandle_ = sim_->sim().getObjectHandle(path); } catch (...) {}
    }
    distances = { checkSonar(sonarHandles_[0]),
                  checkSonar(sonarHandles_[1]),
                  checkSonar(sonarHandles_[2]) };

    isObstacle = false;
    double minDistance = 999.0;
    for (double d : distances) {
        if (d > 0.0) {
            isObstacle = true;
            if (d < minDistance) minDistance = d;
        }
    }

    lambda_ = isObstacle ? std::max(0.05, minDistance) : 1.0;

    // std::cout << "[AvoidObstacles DEBUG] Sonars L/C/R: (" 
    //           << distances[0] << ", " << distances[1] << ", " << distances[2] << ")"
    //           << " | IsObstacle: " << (isObstacle ? "YES" : "NO")
    //           << " | Lambda: " << lambda_ 
    //           << std::endl;
}


bool AvoidObstacles::checkInternalReleaser(){
    return isObstacle;
}

bool AvoidObstacles::checkExternalReleaser(){
    return true;
}

std::pair<double, double> AvoidObstacles::executeMotorSchema() {
    double dLeft   = distances[0];
    double dCenter = distances[1];
    double dRight  = distances[2];

    double turn = 0.0;
    if (dLeft   > 0.0) turn -= 1.0 / (dLeft   + 0.1);
    if (dRight  > 0.0) turn += 1.0 / (dRight  + 0.1);
    if (dCenter > 0.0) turn -= 1.0 / (dCenter + 0.1);

    double v = 0.12;
    if (dCenter > 0.0 && dCenter < 0.4) v = 0.05;
    double w = std::clamp(turn * 0.3, -1.1, 1.1);

    double v_left  = v - w * 0.331 / 2.0;
    double v_right = v + w * 0.331 / 2.0;
    return {v_left + 0.5, v_right +0.5};
}