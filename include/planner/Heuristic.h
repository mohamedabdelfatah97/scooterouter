#pragma once
#include "../core/Types.h"
#include "../core/Node.h"
#include <cmath>
#include <functional>

namespace sr {

// Heuristic function type: takes two nodes, returns estimated cost
using HeuristicFn = std::function<Cost(const Node&, const Node&)>;

struct Heuristic {
    // Straight-line geographic distance (default for road graphs)
    static HeuristicFn euclidean();

    // Manhattan distance on projected screen coords
    static HeuristicFn manhattan();

    // Octile distance — good for 8-connected grids
    static HeuristicFn octile();

    // Zero heuristic — turns A* into Dijkstra (used for benchmarking)
    static HeuristicFn zero();

    // Factory: select by name from CLI arg --heuristic
    static HeuristicFn fromString(const std::string& name);
};

} // namespace sr
