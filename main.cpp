#include <iostream>
#include <thread>
#include <chrono>

#include "ReachColor.h"
#include "AvoidObstacles.h"
#include "Wander.h"
#include "GoToXY.h"
#include "Place.h"
#include "MotorArbitrator.h"
#include <behaviortree_cpp/bt_factory.h>

#include "RemoteAPIClient.h"
#include "RemoteAPIObjects.h"


int main(int argc, char** argv) {

    RemoteAPIClient client("127.0.0.1", 23000);
    RemoteAPIObjects sim(&client);

    BT::BehaviorTreeFactory factory;

    factory.registerNodeType<AvoidObstacles>("AvoidObstacles", &sim);
    factory.registerNodeType<ReachColor>("ReachColor", &sim);
    factory.registerNodeType<GoToXY>("GoToXY", &sim);
    factory.registerNodeType<Place>("Place", &sim);
    factory.registerNodeType<Wander>("Wander", &sim);

    auto tree = factory.createTreeFromFile("../plan/case1.xml");

    int64_t leftMotor = sim.sim().getObjectHandle("/PioneerP3DX/leftMotor");
    int64_t rightMotor = sim.sim().getObjectHandle("/PioneerP3DX/rightMotor");

    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    sim.sim().startSimulation();
    while (true) {
        auto t0 = std::chrono::steady_clock::now();

        MotorArbitrator::getInstance().reset();
        BT::NodeStatus status = tree.tickOnce();
        MotorArbitrator::getInstance().applyCommand(&sim, leftMotor, rightMotor);

        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t0).count();
        std::cout << "[MAIN] ciclo: " << ms << " ms\n";

        if (status != BT::NodeStatus::RUNNING) {
            // tree.haltTree(); 
        }
    }
    
    std::cout << "Tree terminated with status: " << status << std::endl;
    sim.sim().stopSimulation();

    return 0;
}