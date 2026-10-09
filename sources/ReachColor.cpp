 #include "ReachColor.h"

ReachColor::ReachColor(const std::string& name, const BT::NodeConfig& config, RemoteAPIObjects* sim)
            : AttentionalBehavior(name, config), sim_(sim) {
                setMu(2.0);
                robotHandle_ = sim_->sim().getObjectHandle("/PioneerP3DX");
            }

BT::PortsList ReachColor::providedPorts(){
    BT::PortsList ports = {
        BT::InputPort<std::string>("Color"),
        BT::InputPort<std::string>("ObjectPath"),
        BT::InputPort<std::vector<int>>("Knapsack"),
        BT::InputPort<bool>("Return")
    };
    auto basePorts = AttentionalBehavior::providedPorts();
    ports.insert(basePorts.begin(), basePorts.end());
    return ports;

}

void ReachColor::updatePerceptualSchema() {
    getInput("ObjectPath", target_);
    int64_t th = sim_->sim().getObjectHandle(target_);
    auto pos = sim_->sim().getObjectPosition(th, robotHandle_);
    relX_ = pos[0]; relY_ = pos[1];
    distance_ = std::hypot(relX_, relY_);
    lambda_ = std::max(0.1, distance_);
}

bool ReachColor::checkInternalReleaser(){
    if(distance_ > 12.0 || distance_ < 0.45) return false;
    return true;
}

bool isObjectInKnapsack(RemoteAPIObjects* sim_, std::string target_,std::vector<int> knapsack){
    int64_t targetHandle = sim_->sim().getObjectHandle(target_);
    int targetHandleInt = static_cast<int>(targetHandle);
    return (std::find(knapsack.begin(), knapsack.end(), targetHandleInt) != knapsack.end());
}

bool isKnapsackFull(std::vector<int> knapsack){
    return (knapsack.size() > 3);
}

bool ReachColor::checkExternalReleaser() {
    getInput("Knapsack", knapsack);
    getInput("ObjectPath", objectPath_);
    
    if (isObjectInKnapsack(sim_, target_, knapsack)) return false; 

    return true;
}
std::pair<double, double> ReachColor::executeMotorSchema() {
    int64_t robotHandle  = sim_->sim().getObjectHandle("/PioneerP3DX");
    int64_t targetHandle = sim_->sim().getObjectHandle(target_);

    double dist  = std::sqrt(relX_ * relX_ + relY_ * relY_);
    double angle = std::atan2(relY_, relX_);

    if (dist <= 0.6) return {0.0, 0.0};

    double v = 0.2 * std::max(0.0, std::cos(angle));  // m/s
    double w = std::clamp(angle * 1.5, -1.5, 1.5);

    double v_left  = v - w * 0.331 / 2.0;
    double v_right = v + w * 0.331 / 2.0;

    // std::cout << "[ReachColor MOTOR] Target: " << target_
    //           << " | Dist: " << dist
    //           << " | AngleDeg: " << (angle * 180.0 / M_PI)
    //           << " | vL: " << v_left << " vR: " << v_right << std::endl;

    return {v_left, v_right};
}