#pragma once
#include "PlannerInterface.h"

namespace sr {

class BFS : public PlannerInterface {
public:
    PlanResult plan(const Graph& graph, NodeId start, NodeId goal) override;
    std::string name() const override { return "BFS"; }
};

} // namespace sr
