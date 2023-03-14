#include <CLI/CLI.hpp>
#include <fmt/core.h>
#include "core/Graph.h"
#include "map/OSMLoader.h"
#include "map/CoordinateProjector.h"
#include "routing/FleetManager.h"
#include "routing/MissionController.h"
#include "routing/RouteOptimizer.h"
#include "routing/PriorityScorer.h"
#include "viz/Renderer.h"
#include "planner/AStar.h"
#include "planner/Heuristic.h"

int main(int argc, char** argv) {
    CLI::App app{"scooterouter — e-scooter collection route planner"};

    std::string map_path   = "data/maps/hamburg.osm";
    std::string fleet_path = "data/scooters/fleet.csv";
    std::string heuristic  = "euclidean";
    int width = 1400, height = 800;

    app.add_option("--map",       map_path,   "Path to .osm map file");
    app.add_option("--fleet",     fleet_path, "Path to fleet CSV");
    app.add_option("--heuristic", heuristic,  "Heuristic: euclidean|manhattan|octile");
    app.add_option("--width",     width,      "Window width");
    app.add_option("--height",    height,     "Window height");

    CLI11_PARSE(app, argc, argv);

    fmt::print("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
    fmt::print("  scooterouter\n");
    fmt::print("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
    fmt::print("  map:      {}\n", map_path);
    fmt::print("  fleet:    {}\n", fleet_path);
    fmt::print("  heuristic:{}\n", heuristic);
    fmt::print("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n\n");

    fmt::print("[1/3] Loading map...\n");
    sr::Graph graph;
    sr::OSMLoader loader;
    if (!loader.load(map_path, graph)) {
        fmt::print("Failed to load map: {}\n", map_path);
        return 1;
    }
    fmt::print("      Bounds: ({:.4f}, {:.4f}) → ({:.4f}, {:.4f})\n\n",
               loader.minBounds().lat, loader.minBounds().lon,
               loader.maxBounds().lat, loader.maxBounds().lon);

    fmt::print("[2/3] Loading fleet...\n");
    sr::FleetManager fleet;
    if (!fleet.loadFromCSV(fleet_path)) {
        fmt::print("Failed to load fleet: {}\n", fleet_path);
        return 1;
    }
    fmt::print("      {} scooters loaded ({} collectible, {} critical)\n\n",
               fleet.all().size(), fleet.collectible().size(), fleet.critical().size());

    // Snap scooters to nearest graph nodes ranked by urgency
    fmt::print("[test] Snapping scooters to graph...\n");
    sr::PriorityScorer scorer;
    sr::LatLon operator_pos = { 53.5505, 9.9937 };
    auto ranked = scorer.ranked(fleet.collectible(), operator_pos);

    std::vector<sr::NodeId> snapped;
    for (const auto& s : ranked) {
        sr::NodeId node = graph.nearestNode(s.geo);
        snapped.push_back(node);
        fmt::print("      scooter {} (battery {}%) -> node {}\n",
                   s.id, s.battery_pct, node);
    }
    fmt::print("\n");

    // Chain A* through top 5 prioritized scooters
    fmt::print("[test] Running multi-stop A* route...\n");
    sr::AStar astar(sr::Heuristic::euclidean());
    sr::NodeId current = graph.nearestNode(operator_pos);

    std::vector<sr::NodeId> full_path;
    double total_cost = 0.0;

    for (int i = 0; i < std::min(5, (int)snapped.size()); ++i) {
        auto result = astar.plan(graph, current, snapped[i]);
        if (!result.path.empty()) {
            full_path.insert(full_path.end(), result.path.begin(), result.path.end());
            total_cost += result.cost;
            current = snapped[i];
        }
    }
    fmt::print("      total route: {} nodes, {:.2f} km\n\n", full_path.size(), total_cost);

    fmt::print("[3/3] Initializing renderer...\n");
    sr::Renderer renderer(width, height);
    if (!renderer.init()) {
        fmt::print("Failed to initialize renderer\n");
        return 1;
    }

    sr::RouteOptimizer optimizer(nullptr);
    sr::MissionController mission(fleet, optimizer, graph);
    renderer.run(mission, graph, fleet, full_path);

    return 0;
}