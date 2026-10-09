#include "MotorArbitrator.h"
#include <iostream>

MotorArbitrator& MotorArbitrator::getInstance() {
    static MotorArbitrator instance;
    return instance;
}

void MotorArbitrator::reset() {
    currentWinner_ = "none";
    maxEmphasis_ = -1.0;
    winningVLeft_ = 0.0;
    winningVRight_ = 0.0;
}


void MotorArbitrator::proposeCommand(const std::string& behaviorName, double emphasis, double vLeft, double vRight) {
    double adjustedEmphasis = emphasis;

    if (behaviorName == previousWinner_ && !previousWinner_.empty() && previousWinner_ != "none")
            adjustedEmphasis += 0.15; 
    

    if (adjustedEmphasis > maxEmphasis_) {
        maxEmphasis_ = adjustedEmphasis;
        currentWinner_ = behaviorName;
        winningVLeft_ = vLeft;
        winningVRight_ = vRight;
    }
}

bool MotorArbitrator::getWinningCommand(double& vLeft, double& vRight, std::string& winnerName, double& winnerEmphasis) const {
    if (maxEmphasis_ < 0.0) {
        //std::cout << "[MotorArbitrator] Nessun comportamento ha proposto comandi validi in questo ciclo." << std::endl;
        return false;
    }
    vLeft = winningVLeft_;
    vRight = winningVRight_;
    winnerName = currentWinner_;
    winnerEmphasis = maxEmphasis_;
    // std::cout << "[MotorArbitrator] Comando finale applicato -> Vincitore: " << currentWinner_ 
    //           << " (Enfasi: " << maxEmphasis_ << ") | Velocita': (" << winningVLeft_ << ", " << winningVRight_ << ")" << std::endl;

    return true;
}

void MotorArbitrator::applyCommand(RemoteAPIObjects* sim, int64_t leftMotor, int64_t rightMotor) {
    double vL = 0.0, vR = 0.0;
    std::string winner;
    double maxEmp = 0.0;

    std::cout << "[MotorAribitrator] Winner: " << currentWinner_ << std::endl;

    if (getWinningCommand(vL, vR, winner, maxEmp)) {
        previousWinner_ = currentWinner_;
        if (leftMotor >= 0 && rightMotor >= 0 && sim != nullptr) {
            try {
                //std::cout << "[MotorAribitrator] vL: " << vL << " vR: " << vR << std::endl;
                sim->sim().setJointTargetVelocity(leftMotor,  vL );
                sim->sim().setJointTargetVelocity(rightMotor, vR );
            } catch (...) {}
        }
    }
}

