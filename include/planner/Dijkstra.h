#pragma once
#include "PlannerInterface.h"

namespace sr {

class Dijkstra : public PlannerInterface {
public:
    PlanResult plan(const Graph& graph, NodeId start, NodeId goal) override;
    std::string name() const override { return "Dijkstra"; }
};

} // namespace sr
