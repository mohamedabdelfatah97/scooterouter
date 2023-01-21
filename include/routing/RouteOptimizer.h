#pragma once
#include "Scooter.h"
#include "PriorityScorer.h"
#include "../core/Graph.h"
#include "../planner/PlannerInterface.h"
#include <vector>
#include <memory>

namespace sr {

class RouteOptimizer {
public:
    explicit RouteOptimizer(std::shared_ptr<PlannerInterface> planner);

    // Compute initial collection order from current fleet + operator position
    std::vector<Scooter> optimize(
        const std::vector<Scooter>& fleet,
        const LatLon& operator_pos,
        const Graph& graph);

    // Called after any fleet state change — runs 2-opt on remaining scooters
    std::vector<Scooter> reoptimize(
        const std::vector<Scooter>& remaining,
        const LatLon& operator_pos,
        const Graph& graph);

private:
    // Step 1: nearest neighbor greedy seeded by priority score
    std::vector<Scooter> nearestNeighbor(
        const std::vector<Scooter>& fleet,
        const LatLon& start,
        const Graph& graph);

    // Step 2: 2-opt improvement — swap pairs to reduce total distance
    void twoOpt(std::vector<Scooter>& route, const Graph& graph);

    double routeDistance(
        const std::vector<Scooter>& route,
        const Graph& graph);

    std::shared_ptr<PlannerInterface> planner_;
    PriorityScorer scorer_;
};

} // namespace sr
