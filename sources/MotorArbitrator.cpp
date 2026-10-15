#include "MotorArbitrator.h"
#include "DecisionLogger.h"
#include "SharedPerception.h"
#include <iostream>

MotorArbitrator& MotorArbitrator::getInstance() {
    static MotorArbitrator instance;
    return instance;

}

void MotorArbitrator::reset() {
    proposals_.clear();
    currentWinner_ = "none";
    maxEmphasis_ = -1.0;
    winningVLeft_ = 0.0;
    winningVRight_ = 0.0;
}



void MotorArbitrator::proposeCommand(const std::string& id, double e,
                                     double vL, double vR, bool exclusive) {
    if (e <= 0.0) return;
    proposals_.push_back({id, e, vL, vR, exclusive});
}

bool MotorArbitrator::getWinningCommand(double& vLeft, double& vRight, std::string& winnerName, double& winnerEmphasis) const {
    if (maxEmphasis_ < 0.0) 
        return false;
    
    vLeft = winningVLeft_;
    vRight = winningVRight_;
    winnerName = currentWinner_;
    winnerEmphasis = maxEmphasis_;
    // std::cout << "[MotorArbitrator] Comando finale applicato -> Vincitore: " << currentWinner_ 
    //           << " (Enfasi: " << maxEmphasis_ << ") | Velocita': (" << winningVLeft_ << ", " << winningVRight_ << ")" << std::endl;

    return true;
}

void MotorArbitrator::applyCommand(RemoteAPIObjects* sim, int64_t leftMotor, int64_t rightMotor) {
    const Proposal* win = nullptr;
    for (auto& p : proposals_)
        if (p.exclusive && (!win || p.emphasis > win->emphasis)) win = &p;

    std::unordered_map<std::string, const Proposal*> uniq;
    for (auto& p : proposals_)
        if (!p.exclusive) { auto& s = uniq[p.id]; if (!s || p.emphasis > s->emphasis) s = &p; }

    double sE = 0, sL = 0, sR = 0; const Proposal* top = nullptr;
    auto add = [&](const Proposal* p) {
        sE += p->emphasis; sL += p->emphasis * p->vL; sR += p->emphasis * p->vR;
        if (!top || p->emphasis > top->emphasis) top = p;
    };
    if (win) add(win);
    for (auto& kv : uniq) add(kv.second);

    double vL = sE > 0 ? sL / sE : 0.0, vR = sE > 0 ? sR / sE : 0.0;
    const std::string winner = top ? top->id : "none";

    if (winner != previousWinner_) {
            previousWinner_ = winner;
            try { sim->sim().setStringSignal("active_behavior", winner); } catch (...) {}
        }
    if (sim != nullptr && leftMotor >= 0 && rightMotor >= 0) {
        try {
            sim->sim().setJointTargetVelocity(leftMotor,  vL / 0.0975);
            sim->sim().setJointTargetVelocity(rightMotor, vR / 0.0975);
        } catch (...) { std::cout << "Exception Banner" << std::endl; }
    }
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

