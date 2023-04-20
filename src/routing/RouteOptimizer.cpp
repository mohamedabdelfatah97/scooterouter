#include "routing/RouteOptimizer.h"
#include "planner/AStar.h"
#include "planner/Heuristic.h"
#include <cmath>
#include <algorithm>
#include <limits>

namespace sr {

RouteOptimizer::RouteOptimizer(std::shared_ptr<PlannerInterface> planner)
    : planner_(std::move(planner)) {}

static double haversine(const LatLon& a, const LatLon& b) {
    double dlat = (b.lat - a.lat) * M_PI / 180.0;
    double dlon = (b.lon - a.lon) * M_PI / 180.0;
    double h = std::sin(dlat/2)*std::sin(dlat/2) +
               std::cos(a.lat*M_PI/180.0) * std::cos(b.lat*M_PI/180.0) *
               std::sin(dlon/2)*std::sin(dlon/2);
    return 6371.0 * 2.0 * std::asin(std::sqrt(h));
}

double RouteOptimizer::routeDistance(const std::vector<Scooter>& route,
                                      const Graph& graph) {
    double total = 0.0;
    for (size_t i = 0; i + 1 < route.size(); ++i)
        total += haversine(route[i].geo, route[i+1].geo);
    return total;
}

std::vector<Scooter> RouteOptimizer::nearestNeighbor(
    const std::vector<Scooter>& fleet,
    const LatLon& start,
    const Graph& graph) {

    std::vector<Scooter> remaining = fleet;
    std::vector<Scooter> ordered;
    LatLon current = start;

    while (!remaining.empty()) {
        double best_dist = std::numeric_limits<double>::max();
        size_t best_idx  = 0;

        for (size_t i = 0; i < remaining.size(); ++i) {
            double d = haversine(current, remaining[i].geo);
            if (d < best_dist) {
                best_dist = d;
                best_idx  = i;
            }
        }

        ordered.push_back(remaining[best_idx]);
        current = remaining[best_idx].geo;
        remaining.erase(remaining.begin() + best_idx);
    }
    return ordered;
}

void RouteOptimizer::twoOpt(std::vector<Scooter>& route, const Graph& graph) {
    bool improved = true;
    while (improved) {
        improved = false;
        for (size_t i = 1; i + 1 < route.size(); ++i) {
            for (size_t j = i + 1; j < route.size(); ++j) {
                double before = haversine(route[i-1].geo, route[i].geo)
                              + haversine(route[j-1].geo, route[j < route.size()-1 ? j : j].geo);
                double after  = haversine(route[i-1].geo, route[j-1].geo)
                              + haversine(route[i].geo,   route[j < route.size()-1 ? j : j].geo);
                if (after < before - 1e-6) {
                    std::reverse(route.begin() + i, route.begin() + j);
                    improved = true;
                }
            }
        }
    }
}

std::vector<Scooter> RouteOptimizer::optimize(
    const std::vector<Scooter>& fleet,
    const LatLon& operator_pos,
    const Graph& graph) {

    auto ordered = nearestNeighbor(fleet, operator_pos, graph);
    twoOpt(ordered, graph);
    return ordered;
}

std::vector<Scooter> RouteOptimizer::reoptimize(
    const std::vector<Scooter>& remaining,
    const LatLon& operator_pos,
    const Graph& graph) {
    return optimize(remaining, operator_pos, graph);
}

} // namespace sr