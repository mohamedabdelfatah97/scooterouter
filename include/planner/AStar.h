#pragma once
#include "PlannerInterface.h"
#include "Heuristic.h"

namespace sr {

class AStar : public PlannerInterface {
public:
    explicit AStar(HeuristicFn heuristic = Heuristic::euclidean());

    PlanResult plan(const Graph& graph, NodeId start, NodeId goal) override;
    std::string name() const override { return "A*"; }

    // Expose frontier trace for FrontierLayer visualization
    const std::vector<NodeId>& lastExpandedNodes() const;

private:
    HeuristicFn heuristic_;
    std::vector<NodeId> expanded_trace_;
};

} // namespace sr
