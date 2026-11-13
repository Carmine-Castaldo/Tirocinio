#ifndef MOTOR_COMMAND_H
#define MOTOR_COMMAND_H

#include <string>
#include <utility>
#include <unordered_map>
#include <memory>
#include <mutex>

struct MotorCommand {
    std::string owner = "";
    double emphasis = -1.0;
    std::pair<double, double> command = {0.0, 0.0};

    static std::mutex& lock() { static std::mutex m; return m; }

    // TO TEST NYQUISIT-SHANNON THEOREM  
    static inline std::atomic<uint64_t> arbReadsTotal{0};
    static inline std::atomic<uint64_t> nodeWritesTotal{0};
    static inline std::atomic<uint64_t> winnerUpdatesTotal{0};
};

#endif // MOTOR_COMMAND_H