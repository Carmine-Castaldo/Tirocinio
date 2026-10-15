#pragma once
#include <fstream>
#include <iomanip>
#include <string>
#include <utility>
#include <vector>

class DecisionLogger {
public:
    static DecisionLogger& get() { static DecisionLogger l; return l; }

    void open(const std::string& path) {
        out_.open(path, std::ios::out | std::ios::trunc);
        if (out_.is_open()) out_ << "t,winner,e_winner,competitors\n";
    }

    void log(double t, const std::string& winner, double eWinner,
             const std::vector<std::pair<std::string, double>>& competitors) {
        if (!out_.is_open()) return;
        out_ << std::fixed << std::setprecision(2) << t << ',' << winner << ',' << eWinner << ",\"";
        for (size_t i = 0; i < competitors.size(); ++i) {
            if (i) out_ << ';';
            out_ << competitors[i].first << '=' << competitors[i].second;
        }
        out_ << "\"\n";
    }

    void close() { if (out_.is_open()) out_.close(); }

private:
    std::ofstream out_;
};