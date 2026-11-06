#ifndef MOTOR_ARBITRATOR_H
#define MOTOR_ARBITRATOR_H

#include <string>
#include <vector>
#include <map>
#include <mutex>
#include <iostream>
#include "RemoteAPIObjects.h"

struct Proposal {
    std::string id;
    double emphasis;
    double vL;
    double vR;
    bool exclusive;
};

class MotorArbitrator {
private:
    std::string previousWinner_ = "none";
    std::string currentWinner_ = "none";
    double maxEmphasis_ = -1.0;
    double winningVLeft_ = 0.0;
    double winningVRight_ = 0.0;

    mutable std::mutex m_;
    std::map<uint64_t, Proposal> props_;
    std::vector<Proposal> proposals_; 

    MotorArbitrator() = default;

public:
    static MotorArbitrator& getInstance();

    void reset();
    void propose(uint64_t key, const std::string& id, double e, double vL, double vR, bool exclusive);
    void withdraw(uint64_t key);

    bool getWinningCommand(double& vLeft, double& vRight, std::string& winnerName, double& winnerEmphasis) const;
    void applyCommand(RemoteAPIObjects* sim, int64_t leftMotor, int64_t rightMotor);
};

#endif // MOTOR_ARBITRATOR_H