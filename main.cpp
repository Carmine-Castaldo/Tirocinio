#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <chrono>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

#include "AvoidObstacles.h"
#include "DecisionLogger.h"
#include "GoToXY.h"
#include "MotorArbitrator.h"
#include "Place.h"
#include "ReachColor.h"
#include "SharedPerception.h"
#include "TrialSetup.h"
#include "Wander.h"

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/bt_observer.h>

#include "RemoteAPIClient.h"
#include "RemoteAPIObjects.h"

const std::vector<std::string> G_CUBE_LIST = {"/GreenCube", "/RedCube", "/YellowCube", "/OrangeCube"};
const std::vector<std::string> G_IGNORE_LIST = {"/PioneerP3DX", "/Bill"};
const std::string G_ROBOT_NAME = "/PioneerP3DX";
const std::string G_BILL_NAME = "/Bill";
static uint64_t g_tickCount = 0;
using clk = std::chrono::steady_clock;
inline long long ms(clk::time_point a, clk::time_point b) {
    return std::chrono::duration_cast<std::chrono::milliseconds>(b - a).count();
}

struct SimHandles {
    int64_t leftMotor  = -1;
    int64_t rightMotor = -1;
    int64_t robot      = -1;
    int64_t camera     = -1;
};

SimHandles initSimulation(RemoteAPIObjects& sim);
BT::Tree createBehaviorTree(BT::BehaviorTreeFactory& factory, RemoteAPIObjects& sim, const std::string& xmlPath);
void setupEnvironmentAndBlackboard(RemoteAPIObjects& sim, const BT::Blackboard::Ptr& blackboard, unsigned seed, const std::string& outDir);
std::pair<std::string, double> runSimulationLoop(RemoteAPIClient& client, RemoteAPIObjects& sim, BT::Tree& tree, 
                                                 const SimHandles& handles, double maxT);
void logFailures(const BT::Tree& tree, const BT::TreeObserver& observer, const std::string& outDir, unsigned seed);
void saveTrialSummary(const std::string& outDir, unsigned seed, const std::string& result, double simTime, double pathLen);


int main(int argc, char** argv) {
    const unsigned seed   = (argc > 1) ? static_cast<unsigned>(std::stoul(argv[1])) : 1;
    const std::string out = (argc > 2) ? argv[2] : "results";
    const double maxT     = (argc > 3) ? std::stod(argv[3]) : 900.0;
    trialSeed() = seed;

    RemoteAPIClient client("127.0.0.1", 23000);
    RemoteAPIObjects sim(&client);

    SimHandles handles = initSimulation(sim);

    BT::BehaviorTreeFactory factory;
    auto tree = createBehaviorTree(factory, sim, "../plan/case1.xml");
    BT::TreeObserver observer(tree);
    auto blackboard = tree.rootBlackboard();

    setupEnvironmentAndBlackboard(sim, blackboard, seed, out);

    DecisionLogger::get().open(out + "/decisions_seed" + std::to_string(seed) + ".csv");
    sim.sim().startSimulation();

    auto [result, pathLen] = runSimulationLoop(client, sim, tree, handles, maxT);

    if (result != "SUCCESS") {
        logFailures(tree, observer, out, seed);
    }

    const double tEnd = SharedPerception::get().time();
    tree.haltTree();
    DecisionLogger::get().close();

    saveTrialSummary(out, seed, result, tEnd, pathLen);

    sim.sim().stopSimulation();

    return 0;
}



SimHandles initSimulation(RemoteAPIObjects& sim) {
    SimHandles h;
    h.leftMotor  = sim.sim().getObjectHandle((G_ROBOT_NAME + "/leftMotor").c_str());
    h.rightMotor = sim.sim().getObjectHandle((G_ROBOT_NAME + "/rightMotor").c_str());
    h.robot      = sim.sim().getObjectHandle(G_ROBOT_NAME.c_str());
    h.camera     = sim.sim().getObjectHandle((G_ROBOT_NAME + "/visionSensor").c_str());
    
    SharedPerception::get().setup(&sim, h.robot, h.camera);
    return h;
}

BT::Tree createBehaviorTree(BT::BehaviorTreeFactory& factory, RemoteAPIObjects& sim, const std::string& xmlPath) {
    factory.registerNodeType<AvoidObstacles>("AvoidObstacles", &sim);
    factory.registerNodeType<ReachColor>("ReachColor", &sim);
    factory.registerNodeType<GoToXY>("GoToXY", &sim);
    factory.registerNodeType<Place>("Place", &sim);
    factory.registerNodeType<Wander>("Wander", &sim);
    factory.registerNodeType<MotorArbitrator>("MotorArbitrator");
    
    return factory.createTreeFromFile(xmlPath);
}

void setupEnvironmentAndBlackboard(RemoteAPIObjects& sim, const BT::Blackboard::Ptr& blackboard, unsigned seed, const std::string& outDir) {
    auto cubes = randomizeCubes(sim, seed, G_CUBE_LIST, G_IGNORE_LIST);
    {
        std::ofstream f(outDir + "/cubes_seed" + std::to_string(seed) + ".csv");
        f << "cube,x,y\n";
        for (const auto& c : cubes) {
            f << c.path << ',' << c.x << ',' << c.y << '\n';
        }
    }

    for (const auto& c : cubes) {
        std::string cleanName = c.path.substr(1);         
        blackboard->set(cleanName + "_x", c.x);
        blackboard->set(cleanName + "_y", c.y);
    }

    int64_t billHandle = sim.sim().getObjectHandle(G_BILL_NAME.c_str());
    auto posBill = sim.sim().getObjectPosition(billHandle, -1);
    blackboard->set("Bill_x", posBill[0]);
    blackboard->set("Bill_y", posBill[1]);

    blackboard->set("Motor", MotorCommand{});
}

std::pair<std::string, double> runSimulationLoop(RemoteAPIClient& client, RemoteAPIObjects& sim, BT::Tree& tree, 
                                                 const SimHandles& handles, double maxT) {
    std::string result = "TIMEOUT";
    double pathLen = 0.0, px = 0.0, py = 0.0;
    bool havePrev = false;

    auto& sp = SharedPerception::get();
    double lastSimTime = -1.0;



    tree.tickExactlyOnce();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    while (true) {
        const double simNow = sp.exec([](RemoteAPIObjects& s) { return s.sim().getSimulationTime(); });
        if (simNow <= lastSimTime) {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            continue;
        }

        lastSimTime = simNow;
        g_tickCount++;

        const uint64_t reads  = MotorCommand::arbReadsTotal.load(std::memory_order_relaxed);
        const uint64_t writes = MotorCommand::nodeWritesTotal.load(std::memory_order_relaxed);
        const double ratio    = (writes > 0) ? static_cast<double>(reads) / writes : 0.0;
        std::cout << "[NYQUIST] Letture Arbitrator: " << reads
        << " | Scritture Nodi " << writes
        << " | Rapporto reads / writes = " << ratio << "\n";
        
        const auto t0 = clk::now();
        sp.newTick(simNow);
        
        const auto t1 = clk::now();
        BT::NodeStatus st = tree.tickExactlyOnce();
        const auto t2 = clk::now();
    
        std::this_thread::sleep_until(t0 + std::chrono::milliseconds(10));
        std::cout << "Tick " << g_tickCount << " in " << ms(t1, t2) << " ms" << std::endl; 

        // std::cout << "[PROF] tick " << g_tickCount << " t=" << simNow
        //           << " | newTick=" << ms(t1, t2)
        //           << " waitArb=" << ms(t2, t3) << " treeTick=" << ms(t3, t4)
        //           << " TOTAL=" << ms(t0, t4) << " ms | " << sp.profileReport() << std::endl;

        const double t = sp.time();
        const auto pos = sp.position();
        if (havePrev)
            pathLen += std::hypot(pos[0] - px, pos[1] - py);

        px = pos[0];
        py = pos[1];
        havePrev = true;

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