#include "MotorArbitrator.h"
#include "DecisionLogger.h"
#include "SharedPerception.h"
#include <iostream>
#include <unordered_map>
#include <algorithm>

MotorArbitrator& MotorArbitrator::getInstance() {
    static MotorArbitrator instance;
    return instance;
}

void MotorArbitrator::reset() {
    std::lock_guard<std::mutex> l(m_);
    props_.clear();
    proposals_.clear();
    currentWinner_ = "none";
    maxEmphasis_ = -1.0;
    winningVLeft_ = 0.0;
    winningVRight_ = 0.0;
}

void MotorArbitrator::propose(uint64_t key, const std::string& id, double e, double vL, double vR, bool exclusive) {
    if (e <= 0.0) return;
    std::lock_guard<std::mutex> l(m_);
    props_[key] = {id, e, vL, vR, exclusive};
}

void MotorArbitrator::withdraw(uint64_t key) {
    std::lock_guard<std::mutex> l(m_);
    props_.erase(key);
}

bool MotorArbitrator::getWinningCommand(double& vLeft, double& vRight, std::string& winnerName, double& winnerEmphasis) const {
    std::lock_guard<std::mutex> l(m_);
    if (maxEmphasis_ < 0.0) return false;

    vLeft = winningVLeft_;
    vRight = winningVRight_;
    winnerName = currentWinner_;
    winnerEmphasis = maxEmphasis_;
    return true;
}

void MotorArbitrator::applyCommand(RemoteAPIObjects* sim, int64_t leftMotor, int64_t rightMotor) {
    {
        std::lock_guard<std::mutex> l(m_);
        proposals_.clear();
        for (const auto& kv : props_) {
            proposals_.push_back(kv.second);
        }
    }

    if (proposals_.empty()) return;

    const Proposal* win = nullptr;
    for (auto& p : proposals_) {
        if (p.exclusive && (!win || p.emphasis > win->emphasis)) {
            win = &p;
        }
    }

    std::unordered_map<std::string, const Proposal*> uniq;
    for (auto& p : proposals_) {
        if (!p.exclusive) {
            auto& s = uniq[p.id];
            if (!s || p.emphasis > s->emphasis) {
                s = &p;
            }
        }
    }

    double sE = 0.0, sL = 0.0, sR = 0.0;
    const Proposal* top = nullptr;

    auto add = [&](const Proposal* p) {
        sE += p->emphasis;
        sL += p->emphasis * p->vL;
        sR += p->emphasis * p->vR;
        if (!top || p->emphasis > top->emphasis) top = p;
    };

    if (win) add(win);
    for (auto& kv : uniq) add(kv.second);

    double vL = (sE > 0.0) ? (sL / sE) : 0.0;
    double vR = (sE > 0.0) ? (sR / sE) : 0.0;
    const std::string winner = top ? top->id : "none";

     SharedPerception::get().exec([&](RemoteAPIObjects& s) {
        if (winner != previousWinner_) {
            previousWinner_ = winner;
            std::cout << "[Motor Arbitrator] Winner: " << winner << std::endl;
            try { s.sim().setStringSignal("active_behavior", winner); } catch (...) {}
        }

        if (leftMotor >= 0 && rightMotor >= 0) {
            try {
                s.sim().setJointTargetVelocity(leftMotor,  vL / 0.0975);
                s.sim().setJointTargetVelocity(rightMotor, vR / 0.0975);
            } catch (...) {
                std::cout << "Exception in MotorArbitrator::applyCommand" << std::endl;
            }
        }
    });

    if (win) {
        static std::string prevGoal;
        if (win->id != prevGoal) {
            prevGoal = win->id;
            std::map<std::string, double> best;
            for (const auto& p : proposals_)
                if (p.exclusive) best[p.id] = std::max(best[p.id], p.emphasis);
            std::vector<std::pair<std::string, double>> comp(best.begin(), best.end());
            DecisionLogger::get().log(SharedPerception::get().time(), win->id, win->emphasis, comp);
        }
    }
}