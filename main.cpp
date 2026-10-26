#include <cmath>
#include <fstream>
#include <iostream>
#include <string>

#include "ReachColor.h"
#include "AvoidObstacles.h"
#include "Wander.h"
#include "GoToXY.h"
#include "Place.h"
#include "MotorArbitrator.h"
#include "SharedPerception.h"
#include "TrialSetup.h"
#include "DecisionLogger.h"
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/bt_observer.h>

#include "RemoteAPIClient.h"
#include "RemoteAPIObjects.h"


int main(int argc, char** argv) {
    const unsigned seed   = (argc > 1) ? static_cast<unsigned>(std::stoul(argv[1])) : 1;
    const std::string out = (argc > 2) ? argv[2] : "results";
    const double maxT     = (argc > 3) ? std::stod(argv[3]) : 900.0;
    trialSeed() = seed;

    RemoteAPIClient client("127.0.0.1", 23000);
    RemoteAPIObjects sim(&client);
    client.setStepping(true);

    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<AvoidObstacles>("AvoidObstacles", &sim);
    factory.registerNodeType<ReachColor>("ReachColor", &sim);
    factory.registerNodeType<GoToXY>("GoToXY", &sim);
    factory.registerNodeType<Place>("Place", &sim);
    factory.registerNodeType<Wander>("Wander", &sim);
    auto tree = factory.createTreeFromFile("../plan/case1.xml");
    BT::TreeObserver observer(tree); 
    int64_t leftMotor  = sim.sim().getObjectHandle("/PioneerP3DX/leftMotor");
    int64_t rightMotor = sim.sim().getObjectHandle("/PioneerP3DX/rightMotor");
    int64_t robot      = sim.sim().getObjectHandle("/PioneerP3DX");
    int64_t camera     = sim.sim().getObjectHandle("/PioneerP3DX/visionSensor");
    SharedPerception::get().setup(&sim, robot, camera);

    auto cubes = randomizeCubes(sim, seed,
                                {"/GreenCube", "/RedCube", "/YellowCube", "/OrangeCube"},
                                {"/PioneerP3DX", "/Bill"});
    {
        std::ofstream f(out + "/cubes_seed" + std::to_string(seed) + ".csv");
        f << "cube,x,y\n";
        for (const auto& c : cubes) f << c.path << ',' << c.x << ',' << c.y << '\n';
    }
    auto blackboard = tree.rootBlackboard();
    for (const auto& c : cubes) {
        std::string cleanName = c.path.substr(1);         
        blackboard->set(cleanName + "_x", c.x);
        blackboard->set(cleanName + "_y", c.y);
    }
    int64_t billHandle = sim.sim().getObjectHandle("/Bill");
    auto posBill = sim.sim().getObjectPosition(billHandle, -1);
    blackboard->set("Bill_x", posBill[0]);
    blackboard->set("Bill_y", posBill[1]);
    DecisionLogger::get().open(out + "/decisions_seed" + std::to_string(seed) + ".csv");

    sim.sim().startSimulation();
    client.step();

    std::string result = "TIMEOUT";
    double pathLen = 0.0, px = 0.0, py = 0.0;
    bool havePrev = false;
    BT::NodeStatus st;

    
    while (true) {
        SharedPerception::get().newTick();
        sim.sim().handleVisionSensor(camera); 
        const double t = SharedPerception::get().time();
        const auto& pos = SharedPerception::get().position();
        if (havePrev) pathLen += std::hypot(pos[0] - px, pos[1] - py);
        px = pos[0]; py = pos[1]; havePrev = true;

        MotorArbitrator::getInstance().reset();
        st = tree.tickExactlyOnce();
        MotorArbitrator::getInstance().applyCommand(&sim, leftMotor, rightMotor);
        client.step();

        if (st != BT::NodeStatus::RUNNING) {
            result = (st == BT::NodeStatus::SUCCESS) ? "SUCCESS" : "FAILURE";
            break;
        }
        if (t > maxT) break;
    }

    if (result != "SUCCESS") {
        std::vector<std::tuple<long long, std::string, unsigned>> rows;
        tree.applyVisitor([&](const BT::TreeNode* n) {
            const auto& ns = observer.getStatistics(n->UID());
            if (ns.last_result == BT::NodeStatus::FAILURE)
                rows.emplace_back(ns.last_timestamp.count(), n->fullPath(), ns.failure_count);
        });
        std::sort(rows.rbegin(), rows.rend());            // i più recenti per primi
        std::ofstream f(out + "/failures_seed" + std::to_string(seed) + ".txt");
        for (size_t i = 0; i < rows.size() && i < 15; ++i)
            f << std::get<0>(rows[i]) << " ms  " << std::get<1>(rows[i])
            << "  failures=" << std::get<2>(rows[i]) << '\n';
    }

    const double tEnd = SharedPerception::get().time();
    tree.haltTree();
    DecisionLogger::get().close();

    const std::string summary = out + "/summary.csv";
    const bool exists = std::ifstream(summary).good();
    std::ofstream s(summary, std::ios::app);

    if (!exists) s << "seed,result,sim_time_s,path_m\n";
    s << seed << ',' << result << ',' << tEnd << ',' << pathLen << '\n';
    std::cout << "[TRIAL " << seed << "] " << result << " t=" << tEnd << "s path=" << pathLen << "m\n";

    client.setStepping(false);
    sim.sim().stopSimulation();

    return 0;
}