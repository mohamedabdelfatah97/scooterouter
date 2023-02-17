#include "routing/RouteOptimizer.h"

namespace sr {

RouteOptimizer::RouteOptimizer(std::shared_ptr<PlannerInterface> planner)
    : planner_(std::move(planner)) {}

std::vector<Scooter> RouteOptimizer::optimize(
    const std::vector<Scooter>& fleet,
    const LatLon& operator_pos,
    const Graph& graph) { return fleet; }

std::vector<Scooter> RouteOptimizer::reoptimize(
    const std::vector<Scooter>& remaining,
    const LatLon& operator_pos,
    const Graph& graph) { return remaining; }

std::vector<Scooter> RouteOptimizer::nearestNeighbor(
    const std::vector<Scooter>& fleet,
    const LatLon& start,
    const Graph& graph) { return fleet; }

void RouteOptimizer::twoOpt(std::vector<Scooter>& route, const Graph& graph) {}

double RouteOptimizer::routeDistance(
    const std::vector<Scooter>& route,
    const Graph& graph) { return 0.0; }

} // namespace sr