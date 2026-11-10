#pragma once

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "RemoteAPIObjects.h"

class SharedPerception {
public:
    using Sonar = std::tuple<int64_t, double, std::vector<double>, int64_t, std::vector<double>>;

    // Singleton: impedisce copie accidentali
    static SharedPerception& get() {
        static SharedPerception instance;
        return instance;
    }
    SharedPerception(const SharedPerception&) = delete;
    SharedPerception& operator=(const SharedPerception&) = delete;

    void setup(RemoteAPIObjects* sim, int64_t robot, int64_t camera) {
        std::lock_guard<std::recursive_mutex> lock(m_);
        sim_    = sim;
        robot_  = robot;
        camera_ = camera;
    }

    template <class F>
    auto exec(F&& f) {
        std::lock_guard<std::recursive_mutex> lock(m_);
        return f(*sim_);
    }

    int64_t robotHandle() {
        std::lock_guard<std::recursive_mutex> lock(m_);
        return robot_;
    }

    int64_t cameraHandle() {
        std::lock_guard<std::recursive_mutex> lock(m_);
        return camera_;
    }


    void newTick() {
        std::lock_guard<std::recursive_mutex> lock(m_);
        poseOk_ = false;
        treeOk_ = false;
        timeOk_ = false;
        sonar_.clear();

        refreshPose();
        t_      = sim_->sim().getSimulationTime();
        timeOk_ = true;

        updateBestGoal();

        ++seq_;
        done_    = 0;
        arbDone_ = false;
        cv_.notify_all();
    }

    void wake() {
        std::lock_guard<std::recursive_mutex> lock(m_);
        cv_.notify_all();
        cvDone_.notify_all();
    }

    bool waitNextTick(uint64_t& seen, const std::function<bool()>& stop) {
        std::unique_lock<std::recursive_mutex> lock(m_);
        cv_.wait_for(lock, std::chrono::milliseconds(5), [&] {
            return seq_ > seen || stop();
        });
        if (seq_ > seen) {
            seen = seq_;
            return true;
        }
        return false;
    }

    void registerWorker() {
        std::lock_guard<std::recursive_mutex> lock(m_);
        ++workers_;
    }

    void unregisterWorker() {
        std::lock_guard<std::recursive_mutex> lock(m_);
        --workers_;
        cvDone_.notify_all();
    }

    void workerDone() {
        std::lock_guard<std::recursive_mutex> lock(m_);
        ++done_;
        cvDone_.notify_all();
    }

    void waitWorkersDone(std::chrono::milliseconds timeout = std::chrono::milliseconds(50)) {
        std::unique_lock<std::recursive_mutex> lock(m_);
        cvDone_.wait_for(lock, timeout, [&] { return done_ >= workers_; });
    }

    void arbitratorDone() {
        std::lock_guard<std::recursive_mutex> lock(m_);
        arbDone_ = true;
        cvDone_.notify_all();
    }

    void waitArbitratorDone(std::chrono::milliseconds timeout = std::chrono::milliseconds(50)) {
        std::unique_lock<std::recursive_mutex> lock(m_);
        cvDone_.wait_for(lock, timeout, [&] { return arbDone_; });
    }


    std::vector<double> position() {
        std::lock_guard<std::recursive_mutex> lock(m_);
        refreshPose();
        return pos_;
    }

    std::vector<double> orientation() {
        std::lock_guard<std::recursive_mutex> lock(m_);
        refreshPose();
        return ori_;
    }

    double time() {
        std::lock_guard<std::recursive_mutex> lock(m_);
        if (!timeOk_) {
            t_      = sim_->sim().getSimulationTime();
            timeOk_ = true;
        }
        return t_;
    }

    Sonar sonar(int64_t h) {
        std::lock_guard<std::recursive_mutex> lock(m_);
        auto it = sonar_.find(h);
        if (it == sonar_.end()) {
            it = sonar_.emplace(h, sim_->sim().readProximitySensor(h)).first;
        }
        return it->second;
    }

    bool inRobotTree(int64_t h) {
        std::lock_guard<std::recursive_mutex> lock(m_);
        if (!treeOk_) {
            tree_.clear();
            for (int64_t obj : sim_->sim().getObjectsInTree(robot_))
                tree_.insert(obj);
            
            treeOk_ = true;
        }
        return tree_.count(h) > 0;
    }

    std::shared_ptr<const std::vector<unsigned char>> image() {
        std::lock_guard<std::recursive_mutex> lock(m_);
        if (imgSeq_ != seq_ || !img_) {
            auto r  = sim_->sim().getVisionSensorImg(camera_);
            img_    = std::make_shared<std::vector<unsigned char>>(std::move(std::get<0>(r)));
            imgSeq_ = seq_;
        }
        return img_;
    }

    int64_t handle(const std::string& path) {
        std::lock_guard<std::recursive_mutex> lock(m_);
        auto it = handles_.find(path);
        if (it != handles_.end())
            return it->second;

        const int64_t h = sim_->sim().getObjectHandle(path);
        handles_[path]  = h;
        return h;
    }

    std::vector<double> objectPosition(int64_t h, int64_t relativeTo) {
        std::lock_guard<std::recursive_mutex> lock(m_);
        return sim_->sim().getObjectPosition(h, relativeTo);
    }

    double distanceTo(const std::string& path) {
        std::lock_guard<std::recursive_mutex> lock(m_);
        return sim_->getDistanceTo(path);
    }

    void offerGoal(uint64_t key, int64_t handle, double emphasis) {
        std::lock_guard<std::recursive_mutex> lock(m_);
        if (handle >= 0)
            offers_[key] = {handle, emphasis};
    }

    void withdrawGoal(uint64_t key) {
        std::lock_guard<std::recursive_mutex> lock(m_);
        offers_.erase(key);
    }

    bool isGoal(int64_t detected) {
        std::lock_guard<std::recursive_mutex> lock(m_);
        if (goal_ < 0 || detected < 0) return false;
        if (detected == goal_) return true;

        if (goalTreeFor_ != goal_) {
            goalTree_.clear();
            try {
                for (int64_t obj : sim_->sim().getObjectsInTree(goal_)) {
                    goalTree_.insert(obj);
                }
            } catch (...) {}
            goalTreeFor_ = goal_;
        }
        return goalTree_.count(detected) > 0;
    }

private:
    struct GoalOffer {
        int64_t handle  = -1;
        double emphasis = -1.0;
    };

    SharedPerception() = default;

    void refreshPose() {
        if (poseOk_) return;
        pos_    = sim_->sim().getObjectPosition(robot_, -1);
        ori_    = sim_->sim().getObjectOrientation(robot_, -1);
        poseOk_ = true;
    }

    void updateBestGoal() {
        goal_ = -1;
        double bestEmphasis = -1.0;
        for (const auto& [uid, offer] : offers_) {
            if (offer.emphasis > bestEmphasis) {
                bestEmphasis = offer.emphasis;
                goal_        = offer.handle;
            }
        }
    }

    std::recursive_mutex m_;
    std::condition_variable_any cv_;
    std::condition_variable_any cvDone_;
    uint64_t seq_ = 0;
    int workers_  = 0;
    int done_     = 0;
    bool arbDone_ = false;

    RemoteAPIObjects* sim_ = nullptr;
    int64_t robot_         = -1;
    int64_t camera_        = -1;

    bool poseOk_ = false;
    bool treeOk_ = false;
    bool timeOk_ = false;
    std::vector<double> pos_;
    std::vector<double> ori_;
    double t_ = 0.0;
    std::unordered_map<int64_t, Sonar> sonar_;
    std::unordered_set<int64_t> tree_;
    std::shared_ptr<std::vector<unsigned char>> img_;
    uint64_t imgSeq_ = 0;

    std::unordered_map<std::string, int64_t> handles_;
    std::unordered_map<uint64_t, GoalOffer> offers_;
    std::unordered_set<int64_t> goalTree_;
    int64_t goal_        = -1;
    int64_t goalTreeFor_ = -2;
};