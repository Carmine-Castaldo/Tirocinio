#ifndef MOTOR_ARBITRATOR_H
#define MOTOR_ARBITRATOR_H

#include <string>
#include <iostream>
#include "RemoteAPIObjects.h"

//Nota: Classe Singleton

class MotorArbitrator {
    private:
        std::string previousWinner_ = "none", currentWinner_ = "none";
        double maxEmphasis_ = -1.0, winningVLeft_ = 0.0, winningVRight_ = 0.0;
        int64_t bannerHandle_ = -1;
        
        MotorArbitrator() = default;

    public:
        static MotorArbitrator& getInstance();

        void reset();

        void proposeCommand(const std::string& behaviorName, double emphasis, double vLeft, double vRight);

        bool getWinningCommand(double& vLeft, double& vRight, std::string& winnerName, double& winnerEmphasis) const;
        
        void applyCommand(RemoteAPIObjects* sim, int64_t leftMotor, int64_t rightMotor);
};

#endif // MOTOR_ARBITRATOR_H
