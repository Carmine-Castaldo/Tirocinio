#pragma once
#include <chrono>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include "RemoteAPIObjects.h"


class SharedPerception {
public:
    using Sonar = std::tuple<int64_t, double, std::vector<double>, int64_t, std::vector<double>>;

    static SharedPerception& get() { static SharedPerception s; return s; }

    void setup(RemoteAPIObjects* sim, int64_t robot, int64_t camera) {
        std::lock_guard<std::recursive_mutex> l(m_);
        sim_ = sim; robot_ = robot; camera_ = camera;
    }

    template <class F>
    auto exec(F&& f) {
        std::lock_guard<std::recursive_mutex> l(m_);
        return f(*sim_);
    }

    void newTick() {
        std::lock_guard<std::recursive_mutex> l(m_);
        poseOk_ = false; treeOk_ = false; timeOk_ = false;
        refreshPose();
        sonar_.clear();
        goal_ = -1;
        double bestE = -1.0;
        for (const auto& kv : offers_)
            if (kv.second.second > bestE) { bestE = kv.second.second; goal_ = kv.second.first; }
        ++seq_;
        done_ = 0;
        arbDone_ = false;
        cv_.notify_all();
    }

    void wake() { cv_.notify_all(); cvDone_.notify_all(); }

    bool waitNextTick(uint64_t& seen, const std::function<bool()>& stop) {
        std::unique_lock<std::recursive_mutex> lk(m_);
        cv_.wait_for(lk, std::chrono::milliseconds(20), [&] { return seq_ > seen || stop(); });
        if (seq_ > seen) { seen = seq_; return true; }
        return false;
    }
    void registerWorker()   { std::lock_guard<std::recursive_mutex> l(m_); ++workers_; }
    void unregisterWorker() { std::lock_guard<std::recursive_mutex> l(m_); --workers_; cvDone_.notify_all(); }
    void workerDone()       { std::lock_guard<std::recursive_mutex> l(m_); ++done_; cvDone_.notify_all(); }
    void waitWorkersDone(std::chrono::milliseconds timeout = std::chrono::milliseconds(500)) {
        std::unique_lock<std::recursive_mutex> lk(m_);
        cvDone_.wait_for(lk, timeout, [&] { return done_ >= workers_; });
    }
    void arbitratorDone() { std::lock_guard<std::recursive_mutex> l(m_); arbDone_ = true; cvDone_.notify_all(); }
    void waitArbitratorDone(std::chrono::milliseconds timeout = std::chrono::milliseconds(500)) {
        std::unique_lock<std::recursive_mutex> lk(m_);
        cvDone_.wait_for(lk, timeout, [&] { return arbDone_; });
    }
    std::vector<double> position() {
        std::lock_guard<std::recursive_mutex> l(m_);
        refreshPose();
        return pos_;
    }
    std::vector<double> orientation() {
        std::lock_guard<std::recursive_mutex> l(m_);
        refreshPose();
        return ori_;
    }
    int64_t cameraHandle() {
        std::lock_guard<std::recursive_mutex> l(m_);
        return camera_;
    }
    double time() {
        std::lock_guard<std::recursive_mutex> l(m_);
        if (!timeOk_) { t_ = sim_->sim().getSimulationTime(); timeOk_ = true; }
        return t_;
    }

    Sonar sonar(int64_t h) {
        std::lock_guard<std::recursive_mutex> l(m_);
        auto it = sonar_.find(h);
        if (it == sonar_.end()) it = sonar_.emplace(h, sim_->sim().readProximitySensor(h)).first;
        return it->second;
    }

    bool inRobotTree(int64_t h) {
        std::lock_guard<std::recursive_mutex> l(m_);
        if (!treeOk_) {
            tree_.clear();
            for (auto x : sim_->sim().getObjectsInTree(robot_)) tree_.insert(x);
            treeOk_ = true;
        }
        return tree_.count(h) > 0;
    }

    std::shared_ptr<const std::vector<unsigned char>> image() {
        std::lock_guard<std::recursive_mutex> l(m_);
        if (imgSeq_ != seq_ || !img_) {
            auto r = sim_->sim().getVisionSensorImg(camera_);
            const auto& v = std::get<0>(r);
            img_ = std::make_shared<std::vector<unsigned char>>(v.begin(), v.end());
            imgSeq_ = seq_;
        }
        return img_;
    }

    int64_t handle(const std::string& path) {
        std::lock_guard<std::recursive_mutex> l(m_);
        auto it = handles_.find(path);
        if (it != handles_.end()) return it->second;
        const int64_t h = sim_->sim().getObjectHandle(path);
        handles_[path] = h;
        return h;
    }

    std::vector<double> objectPosition(int64_t h, int64_t relativeTo) {
        std::lock_guard<std::recursive_mutex> l(m_);
        return sim_->sim().getObjectPosition(h, relativeTo);
    }

    double distanceTo(const std::string& path) {
        std::lock_guard<std::recursive_mutex> l(m_);
        return sim_->getDistanceTo(path);
    }

    void offerGoal(uint64_t key, int64_t h, double e) {
        std::lock_guard<std::recursive_mutex> l(m_);
        if (h >= 0) offers_[key] = {h, e};
    }
    void withdrawGoal(uint64_t key) {
        std::lock_guard<std::recursive_mutex> l(m_);
        offers_.erase(key);
    }
    bool isGoal(int64_t det) {
        std::lock_guard<std::recursive_mutex> l(m_);
        if (goal_ < 0 || det < 0) return false;
        if (det == goal_) return true;
        if (goalTreeFor_ != goal_) {
            goalTree_.clear();
            try { for (auto x : sim_->sim().getObjectsInTree(goal_)) goalTree_.insert(x); } catch (...) {}
            goalTreeFor_ = goal_;
        }
        return goalTree_.count(det) > 0;
    }

private:
    SharedPerception() = default;

    void refreshPose() { 
        if (poseOk_) return;
        pos_ = sim_->sim().getObjectPosition(robot_, -1);
        ori_ = sim_->sim().getObjectOrientation(robot_, -1);
        poseOk_ = true;
    }

    std::recursive_mutex m_;
    std::condition_variable_any cv_, cvDone_;
    uint64_t seq_ = 0;
    int workers_ = 0, done_ = 0;

    RemoteAPIObjects* sim_ = nullptr;
    int64_t robot_ = -1, camera_ = -1;

    bool poseOk_ = false, treeOk_ = false, timeOk_ = false, arbDone_ = false;
    std::vector<double> pos_, ori_;
    double t_ = 0.0;
    std::unordered_map<int64_t, Sonar> sonar_;
    std::unordered_set<int64_t> tree_;

    std::shared_ptr<std::vector<unsigned char>> img_;
    uint64_t imgSeq_ = 0;
    std::unordered_map<std::string, int64_t> handles_;
    std::unordered_map<uint64_t, std::pair<int64_t, double>> offers_;
    std::unordered_set<int64_t> goalTree_;
    int64_t goal_ = -1, goalTreeFor_ = -2;
};