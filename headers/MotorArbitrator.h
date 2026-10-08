#ifndef MOTOR_ARBITRATOR_H
#define MOTOR_ARBITRATOR_H

#include <string>
#include <iostream>

//Nota: Classe Singleton

class MotorArbitrator {
    private:
        std::string winningBehavior_ = "none";
        double maxEmphasis_ = -1.0;
        double winningVLeft_ = 0.0;
        double winningVRight_ = 0.0;

        MotorArbitrator() = default;

    public:
        static MotorArbitrator& getInstance() {
            static MotorArbitrator instance;
            return instance;
        }

        void reset() {
            winningBehavior_ = "none";
            maxEmphasis_ = -1.0;
            winningVLeft_ = 0.0;
            winningVRight_ = 0.0;
        }

        void proposeCommand(const std::string& behaviorName, double emphasis, double vLeft, double vRight) {
            if (emphasis > maxEmphasis_) {
                maxEmphasis_ = emphasis;
                winningBehavior_ = behaviorName;
                winningVLeft_ = vLeft;
                winningVRight_ = vRight;
            }
        }

        bool getWinningCommand(double& vLeft, double& vRight, std::string& winnerName, double& winnerEmphasis) const {
            if (maxEmphasis_ < 0.0) return false;
            vLeft = winningVLeft_;
            vRight = winningVRight_;
            winnerName = winningBehavior_;
            winnerEmphasis = maxEmphasis_;
            return true;
        }
};

#endif // MOTOR_ARBITRATOR_H
