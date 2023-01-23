#pragma once
#include "../core/Graph.h"

namespace sr {

// Applies cost inflation around blocked edges.
// Radius controls how many hops out from the obstacle to inflate.
// Called after operator adds an obstacle at runtime.
class ObstacleInflation {
public:
    explicit ObstacleInflation(int radius = 2, double inflation_factor = 3.0);
    void inflate(Graph& graph, NodeId blocked_from, NodeId blocked_to);

private:
    int    radius_;
    double factor_;
};

} // namespace sr
