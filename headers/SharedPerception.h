#pragma once
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <tuple>
#include "RemoteAPIObjects.h"

class SharedPerception {
public:
    using Sonar = std::tuple<int64_t, double, std::vector<double>, int64_t, std::vector<double>>;

    static SharedPerception& get() { static SharedPerception s; return s; }

    void setup(RemoteAPIObjects* sim, int64_t robot) { sim_ = sim; robot_ = robot; }
    void newTick() { poseOk_ = false; treeOk_ = false; sonar_.clear(); goal_ = bestGoal_;                              // il goal del tick precedente (ritardo di 1 tick: irrilevante)
                     bestGoal_ = -1; bestE_ = -1.0; timeOk_ = false;}

    const std::vector<double>& position()    { refreshPose(); return pos_; }
    const std::vector<double>& orientation() { refreshPose(); return ori_; }

    const Sonar& sonar(int64_t h) { 
        auto it = sonar_.find(h);
        if (it == sonar_.end()) it = sonar_.emplace(h, sim_->sim().readProximitySensor(h)).first;
        return it->second;
    }

    bool inRobotTree(int64_t h) { 
        if (!treeOk_) {
            tree_.clear();
            for (auto x : sim_->sim().getObjectsInTree(robot_)) tree_.insert(x);
            treeOk_ = true;
        }
        return tree_.count(h) > 0;
    }

    void offerGoal(int64_t h, double e) {
        if (h >= 0 && e > bestE_) { bestE_ = e; bestGoal_ = h; }
    }
    bool isGoal(int64_t det) {
        if (goal_ < 0 || det < 0) return false;
        if (det == goal_) return true;
        if (goalTreeFor_ != goal_) {
            goalTree_.clear();
            try { for (auto x : sim_->sim().getObjectsInTree(goal_)) goalTree_.insert(x); } catch (...) {}
            goalTreeFor_ = goal_;
        }
        return goalTree_.count(det) > 0;
    }
    double time() {
        if (!timeOk_) { t_ = sim_->sim().getSimulationTime(); timeOk_ = true; }
        return t_;
    }

private:
    void refreshPose() {
        if (poseOk_) return;
        pos_ = sim_->sim().getObjectPosition(robot_, -1);
        ori_ = sim_->sim().getObjectOrientation(robot_, -1);
        poseOk_ = true;
    }
    RemoteAPIObjects* sim_ = nullptr;
    int64_t robot_ = -1;
    bool poseOk_ = false, treeOk_ = false;
    std::vector<double> pos_, ori_;
    std::unordered_map<int64_t, Sonar> sonar_;
    std::unordered_set<int64_t> tree_;
    int64_t goal_ = -1, bestGoal_ = -1, goalTreeFor_ = -2;
    double  bestE_ = -1.0;
    double t_ = 0.0; bool timeOk_ = false;
    std::unordered_set<int64_t> goalTree_;
};