#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <tuple>
#include <vector>

#include "AvoidObstacles.h"
#include "DecisionLogger.h"
#include "GoToXY.h"
#include "MotorArbitrator.h"
#include "Place.h"
#include "ReachColor.h"
#include "SharedPerception.h"
#include "TaskPriorityManager.h"
#include "TrialSetup.h"
#include "Wander.h"

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/bt_observer.h>

#include "RemoteAPIClient.h"
#include "RemoteAPIObjects.h"

//If the robot/components/objects/human change, change the path here
const std::vector<std::string> objectsPath_ = {"/GreenCube", "/RedCube", "/YellowCube", "/OrangeCube"};
const std::string robotPath_ = "/PioneerP3DX";
const std::string robotLM_ = "/PioneerP3DX/leftMotor";
const std::string robotRM_ = "/PioneerP3DX/rightMotor";
const std::string robotCam_ = "/PioneerP3DX/visionSensor";
const std::string humanPath_ = "/Bill";

struct SimHandles {
    int64_t leftMotor  = -1;
    int64_t rightMotor = -1;
    int64_t robot      = -1;
    int64_t camera     = -1;
};


SimHandles initSimulation(RemoteAPIObjects& sim);
BT::Tree createBehaviorTree(BT::BehaviorTreeFactory& factory, RemoteAPIObjects& sim, const std::string& xmlPath);
std::vector<TaskPriorityManager> createPriorityManagers(const BT::Blackboard::Ptr& blackboard);
void setupEnvironmentAndBlackboard(RemoteAPIObjects& sim, const BT::Blackboard::Ptr& blackboard, unsigned seed, const std::string& outDir);
std::pair<std::string, double> runSimulationLoop(RemoteAPIClient& client, RemoteAPIObjects& sim, BT::Tree& tree, 
                                                 std::vector<TaskPriorityManager>& managers, const SimHandles& handles, double maxT);
void logFailures(const BT::Tree& tree, const BT::TreeObserver& observer, const std::string& outDir, unsigned seed);
void saveTrialSummary(const std::string& outDir, unsigned seed, const std::string& result, double simTime, double pathLen);


int main(int argc, char** argv) {
    const unsigned seed   = (argc > 1) ? static_cast<unsigned>(std::stoul(argv[1])) : 1;
    const std::string out = (argc > 2) ? argv[2] : "results";
    const double maxT     = (argc > 3) ? std::stod(argv[3]) : 900.0;
    trialSeed() = seed;

    RemoteAPIClient client("127.0.0.1", 23000);
    RemoteAPIObjects sim(&client);
    client.setStepping(true);

    SimHandles handles = initSimulation(sim);

    BT::BehaviorTreeFactory factory;
    auto tree = createBehaviorTree(factory, sim, "../plan/case1.xml");
    BT::TreeObserver observer(tree);
    auto blackboard = tree.rootBlackboard();

    auto priorityManagers = createPriorityManagers(blackboard);
    setupEnvironmentAndBlackboard(sim, blackboard, seed, out);

    DecisionLogger::get().open(out + "/decisions_seed" + std::to_string(seed) + ".csv");

    sim.sim().startSimulation();
    client.step();

    auto [result, pathLen] = runSimulationLoop(client, sim, tree, priorityManagers, handles, maxT);

    if (result != "SUCCESS") logFailures(tree, observer, out, seed);
    
    const double tEnd = SharedPerception::get().time();
    tree.haltTree();
    DecisionLogger::get().close();

    saveTrialSummary(out, seed, result, tEnd, pathLen);

    client.setStepping(false);
    sim.sim().stopSimulation();

    return 0;
}

// ######################################################################################################################

SimHandles initSimulation(RemoteAPIObjects& sim) {
    SimHandles handles;
    handles.robot      = sim.sim().getObjectHandle(robotPath_);
    handles.leftMotor  = sim.sim().getObjectHandle(robotLM_);
    handles.rightMotor = sim.sim().getObjectHandle(robotRM_);
    handles.camera     = sim.sim().getObjectHandle(robotCam_);
    
    SharedPerception::get().setup(&sim, handles.robot, handles.camera);
    return handles;
}

BT::Tree createBehaviorTree(BT::BehaviorTreeFactory& factory, RemoteAPIObjects& sim, const std::string& xmlPath) {
    factory.registerNodeType<AvoidObstacles>("AvoidObstacles", &sim);
    factory.registerNodeType<ReachColor>("ReachColor", &sim);
    factory.registerNodeType<GoToXY>("GoToXY", &sim);
    factory.registerNodeType<Place>("Place", &sim);
    factory.registerNodeType<Wander>("Wander", &sim);
    
    return factory.createTreeFromFile(xmlPath);
}

std::vector<TaskPriorityManager> createPriorityManagers(const BT::Blackboard::Ptr& blackboard) {
    TaskPriorityManager managerTask1("Task1", blackboard, {
        {"Take_Yellow",   "mu_t1_1", "done_yellow"},
        {"Take_Orange",   "mu_t1_2", "done_orange"},
        {"Return_Yellow", "mu_t1_3", "placed_yellow"},
        {"Return_Orange", "mu_t1_4", "placed_orange"}
    }, 1.0, 1.0);

    TaskPriorityManager managerTask2("Task2", blackboard, {
        {"Take_Green",    "mu_t2_1", "done_green"},
        {"Take_Red",      "mu_t2_2", "done_red"},
        {"Return_Green",  "mu_t2_3", "placed_green"},
        {"Return_Red",    "mu_t2_4", "placed_red"}
    }, 1.0, 1.0);

    return {managerTask1, managerTask2};
}

void setupEnvironmentAndBlackboard(RemoteAPIObjects& sim, const BT::Blackboard::Ptr& blackboard, unsigned seed, const std::string& outDir) {
    
    auto cubes = randomizeCubes(sim, seed,
                                {"/GreenCube", "/RedCube", "/YellowCube", "/OrangeCube"},
                                {"/PioneerP3DX", "/Bill"});
    {
        std::ofstream f(outDir + "/cubes_seed" + std::to_string(seed) + ".csv");
        f << "cube,x,y\n";
        for (const auto& c : cubes) {
            f << c.path << ',' << c.x << ',' << c.y << '\n';
        }
    }

    for (const auto& c : cubes) {   
        std::string name = c.path.substr(1);
        blackboard->set(name + "_x", c.x);
        blackboard->set(name + "_y", c.y);
    }

    int64_t billHandle = sim.sim().getObjectHandle("/Bill");
    auto posBill = sim.sim().getObjectPosition(billHandle, -1);
    blackboard->set("Bill_x", posBill[0]);
    blackboard->set("Bill_y", posBill[1]);
}

std::pair<std::string, double> runSimulationLoop(RemoteAPIClient& client, RemoteAPIObjects& sim, BT::Tree& tree, 
                                                 std::vector<TaskPriorityManager>& managers, const SimHandles& handles, double maxT) {
    std::string result = "TIMEOUT";
    double pathLen = 0.0, px = 0.0, py = 0.0;
    bool havePrev = false;

    while (true) {
        SharedPerception::get().newTick();
        sim.sim().handleVisionSensor(handles.camera);
        
        const double t = SharedPerception::get().time();
        const auto& pos = SharedPerception::get().position();
        
        if (havePrev) pathLen += std::hypot(pos[0] - px, pos[1] - py);
        px = pos[0]; 
        py = pos[1]; 
        havePrev = true;

        for (auto& manager : managers) manager.update();
        
        MotorArbitrator::getInstance().reset();
        BT::NodeStatus st = tree.tickExactlyOnce();
        MotorArbitrator::getInstance().applyCommand(&sim, handles.leftMotor, handles.rightMotor);
        client.step();

        if (st != BT::NodeStatus::RUNNING) {
            result = (st == BT::NodeStatus::SUCCESS) ? "SUCCESS" : "FAILURE";
            break;
        }
        if (t > maxT) break;
    }

    return {result, pathLen};
}

void logFailures(const BT::Tree& tree, const BT::TreeObserver& observer, const std::string& outDir, unsigned seed) {
    std::vector<std::tuple<long long, std::string, unsigned>> rows;
    tree.applyVisitor([&](const BT::TreeNode* n) {
        const auto& ns = observer.getStatistics(n->UID());
        if (ns.last_result == BT::NodeStatus::FAILURE) {
            rows.emplace_back(ns.last_timestamp.count(), n->fullPath(), ns.failure_count);
        }
    });
    
    std::sort(rows.rbegin(), rows.rend());
    std::ofstream f(outDir + "/failures_seed" + std::to_string(seed) + ".txt");
    for (size_t i = 0; i < rows.size() && i < 15; ++i) {
        f << std::get<0>(rows[i]) << " ms  " << std::get<1>(rows[i])
          << "  failures=" << std::get<2>(rows[i]) << '\n';
    }
}

void saveTrialSummary(const std::string& outDir, unsigned seed, const std::string& result, double simTime, double pathLen) {
    const std::string summary = outDir + "/summary.csv";
    const bool exists = std::ifstream(summary).good();
    std::ofstream s(summary, std::ios::app);

    if (!exists) s << "seed,result,sim_time_s,path_m\n";
    s << seed << ',' << result << ',' << simTime << ',' << pathLen << '\n';
    
    std::cout << "[TRIAL " << seed << "] " << result 
              << " t=" << simTime << "s path=" << pathLen << "m\n";
}




