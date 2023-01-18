#pragma once
#include "../core/Graph.h"
#include <vector>

namespace sr {

struct PlanResult {
    std::vector<NodeId> path;   // ordered node ids from start to goal
    Cost                cost;   // total path cost
    size_t              nodes_expanded = 0;
};

class PlannerInterface {
public:
    virtual ~PlannerInterface() = default;
    virtual PlanResult plan(const Graph& graph, NodeId start, NodeId goal) = 0;
    virtual std::string name() const = 0;
};

} // namespace sr
