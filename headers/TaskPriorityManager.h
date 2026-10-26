#pragma once
#include <behaviortree_cpp/blackboard.h>
#include <vector>
#include <string>
#include <iostream>
#include <iomanip>

struct TaskStep {
    std::string name;     
    std::string mu_port;  
    std::string done_port;
};

class TaskPriorityManager {
public:
    TaskPriorityManager(std::string taskName,
                        BT::Blackboard::Ptr blackboard,
                        std::vector<TaskStep> sequence,
                        double baseMu = 1.0,
                        double prioritizedBonus = 1.0) 
        : taskName_(std::move(taskName)),
          blackboard_(std::move(blackboard)),
          sequence_(std::move(sequence)),
          baseMu_(baseMu),
          prioritizedBonus_(prioritizedBonus)
    {
        update();
    }

    void update() {
        int firstUnfinishedIdx = -1;

        for (size_t i = 0; i < sequence_.size(); ++i) {
            bool isDone = false;
            if (blackboard_->get(sequence_[i].done_port, isDone) && isDone) {
                continue;
            } else {
                firstUnfinishedIdx = static_cast<int>(i);
                break;
            }
        }


        if (firstUnfinishedIdx != currentActiveIndex_) {
            currentActiveIndex_ = firstUnfinishedIdx;

            if (firstUnfinishedIdx != -1) {
                double currentMu = baseMu_;
                (void)blackboard_->get(sequence_[firstUnfinishedIdx].mu_port, currentMu);

                double newMu = currentMu + prioritizedBonus_;
                std::cout << currentMu << std::endl;
                blackboard_->set(sequence_[firstUnfinishedIdx].mu_port, newMu);

                std::cout << "\n================================================================" << std::endl;
                if (firstUnfinishedIdx == 0) {
                    std::cout << "[" << taskName_ << " - START] Priorizited Mangitude To First Behavior In Sequence: "
                              << sequence_[firstUnfinishedIdx].name << " (mu: " << currentMu << " -> " << newMu << ")" << std::endl;
                } else {
                    std::cout << "[" << taskName_ << " - STEP TERMINATED] Incremented Magnitude To Next Behavior In Sequence: "
                              << sequence_[firstUnfinishedIdx].name << " (magnitude: " << currentMu << " -> " << newMu << ")" << std::endl;
                }
                std::cout << "----------------------------------------------------------------" << std::endl;
                printCurrentMuState();
                std::cout << "================================================================\n" << std::endl;
            } else {
                std::cout << "\n[" << taskName_ << " - COMPLETED] !\n" << std::endl;
            }
        }
    }

private:
    void printCurrentMuState() const {
        for (size_t i = 0; i < sequence_.size(); ++i) {
            bool isDone = false;
            (void)blackboard_->get(sequence_[i].done_port, isDone);

            double muVal = baseMu_;
            (void)blackboard_->get(sequence_[i].mu_port, muVal);

            std::string status = isDone ? "COMPLETED" : (static_cast<int>(i) == currentActiveIndex_ ? ">>> ACTIVE BEHAVIOR <<<" : "TO COMPLETE");
            std::cout << "  Step [" << (i + 1) << "] " << std::left << std::setw(15) << sequence_[i].name
                      << " | Port: " << std::setw(8) << sequence_[i].mu_port
                      << " | magnitude = " << muVal << "  (" << status << ")" << std::endl;
        }
    }

    std::string taskName_;
    BT::Blackboard::Ptr blackboard_;
    std::vector<TaskStep> sequence_;
    double baseMu_;
    double prioritizedBonus_;
    int currentActiveIndex_ = -2;
};