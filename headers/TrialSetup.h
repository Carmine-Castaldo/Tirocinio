#pragma once
#include <cmath>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>
#include "RemoteAPIObjects.h"

inline unsigned& trialSeed() { static unsigned s = 1; return s; }

struct CubePose { std::string path; double x, y; };

inline std::vector<CubePose> randomizeCubes(RemoteAPIObjects& sim, unsigned seed,
                                            const std::vector<std::string>& cubePaths,
                                            const std::vector<std::string>& keepOutPaths,
                                            double half = 6.5,
                                            double minCubeDist = 1.5,
                                            double minKeepOut = 2.0)
{
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> U(-half, half);

    std::vector<std::pair<double, double>> keep;
    for (const auto& p : keepOutPaths) {
        auto pos = sim.sim().getObjectPosition(sim.sim().getObjectHandle(p), -1);
        keep.emplace_back(pos[0], pos[1]);
    }

    std::vector<CubePose> placed;
    for (const auto& path : cubePaths) {
        int64_t h = sim.sim().getObjectHandle(path);
        auto cur = sim.sim().getObjectPosition(h, -1);
        bool ok = false;
        for (int attempt = 0; attempt < 2000 && !ok; ++attempt) {
            const double x = U(rng), y = U(rng);
            ok = true;
            for (const auto& k : keep)   if (std::hypot(x - k.first, y - k.second) < minKeepOut) ok = false;
            for (const auto& c : placed) if (std::hypot(x - c.x, y - c.y) < minCubeDist)         ok = false;
            if (ok) {
                placed.push_back({path, x, y});
                sim.sim().setObjectPosition(h, -1, {x, y, cur[2]});
            }
        }
        if (!ok) throw std::runtime_error("randomizeCubes: nessuna posizione valida per " + path);
    }
    return placed;
}