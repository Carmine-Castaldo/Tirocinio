    #include "Place.h"

    Place::Place(const std::string& name, const BT::NodeConfig& config, RemoteAPIObjects* sim)
        : AttentionalBehavior(name, config), sim_(sim) {}

    BT::PortsList Place::providedPorts(){
        BT::PortsList ports = {
            BT::InputPort<std::string>("Object"),
            BT::InputPort<std::string>("Color"),
            BT::InputPort<std::string>("Target"),
            BT::BidirectionalPort<bool>("Done"),
            BT::BidirectionalPort<std::vector<int>>("knapsack")
        };
        auto basePorts = AttentionalBehavior::providedPorts();
        ports.insert(basePorts.begin(), basePorts.end());
        return ports;
    }

    bool Place::checkInternalReleaser(){
        if (targetHandle_ == -1) return false;
        bool inKnapsack = (std::find(knapsack.begin(), knapsack.end(), static_cast<int>(targetHandle_)) != knapsack.end());
        if (target_ == "ground")return inKnapsack;
        else return (knapsack.size() < 4 && !inKnapsack);
    }

    bool Place::checkExternalReleaser(){
        bool isDone = false;
        getInput("Done", isDone);

        if (target_ == "knapsack" && isDone ) return false;      

        return (distance < 0.6);
    }

    void Place::updatePerceptualSchema() {
        getInput("Target", target_);
        getInput("Object", objectPath_);
        getInput("knapsack", knapsack);
        if (!objectPath_.empty() && objectPath_ != previousPath_) {
            targetHandle_ = sim_->sim().getObjectHandle(objectPath_);
            previousPath_ = objectPath_;
        }
        distance = 999.0;
        try { distance = sim_->getDistanceTo(target_ == "ground" ? "/Bill" : objectPath_); }
        catch (...) {}
        lambda_ = saturatingLambda(distance, 0.5, 1.0);
    }

    std::pair<double, double> Place::executeMotorSchema() {
        if (target_ == "knapsack") {
            size_t n = knapsack.size();
            int index = n + 6;
            std::string connectionPath = "/PioneerP3DX/connection[" + std::to_string(index)  + "]";
            
            try {
                int64_t connHandle = sim_->sim().getObjectHandle(connectionPath);
                sim_->sim().setObjectInt32Param(targetHandle_, sim_->sim().shapeintparam_static, 1);
                sim_->sim().setObjectInt32Param(targetHandle_, sim_->sim().shapeintparam_respondable, 0);
                sim_->sim().resetDynamicObject(targetHandle_);
                sim_->sim().setObjectParent(targetHandle_, connHandle, false);
                sim_->sim().setObjectPosition(targetHandle_, connHandle, {0.0, 0.0, 0.05});

                if (std::find(knapsack.begin(), knapsack.end(), static_cast<int>(targetHandle_)) == knapsack.end())
                    knapsack.push_back(static_cast<int>(targetHandle_));
                
                setOutput("Done", true);
                setOutput("knapsack", knapsack);
            } catch (...) {}
        } else if (target_ == "ground") {
            sim_->sim().setObjectParent(targetHandle_, -1, true);
            auto robotPos = sim_->getRobotPosition();
            std::vector<double> groundPos = {robotPos[0] + 0.4, robotPos[1] + 0.3, 0.05};
            sim_->sim().setObjectPosition(targetHandle_, -1, groundPos);
            sim_->sim().setObjectInt32Param(targetHandle_, sim_->sim().shapeintparam_static, 0);
            sim_->sim().setObjectInt32Param(targetHandle_, sim_->sim().shapeintparam_respondable, 1);
            sim_->sim().resetDynamicObject(targetHandle_);

            auto it = std::find(knapsack.begin(), knapsack.end(), static_cast<int>(targetHandle_));
            if (it != knapsack.end()) {
                knapsack.erase(it);
            }
            setOutput("Done", true);
            setOutput("knapsack", knapsack);
        }
        
        return {0.0, 0.0};
    }

