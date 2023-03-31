#include <CLI/CLI.hpp>
#include <fmt/core.h>
#include <random>
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
    std::string fleet_path = "";
    std::string heuristic  = "euclidean";
    int width      = 1400;
    int height     = 800;
    int n_scooters = 15;
    int seed       = -1;

    app.add_option("--map",       map_path,   "Path to .osm map file");
    app.add_option("--fleet",     fleet_path, "Path to fleet CSV (optional)");
    app.add_option("--heuristic", heuristic,  "Heuristic: euclidean|manhattan|octile");
    app.add_option("--width",     width,      "Window width");
    app.add_option("--height",    height,     "Window height");
    app.add_option("--scooters",  n_scooters, "Number of scooters to generate");
    app.add_option("--seed",      seed,       "Random seed (-1 = random each run)");

    CLI11_PARSE(app, argc, argv);

    unsigned int actual_seed = (seed < 0)
        ? std::random_device{}()
        : static_cast<unsigned int>(seed);

    fmt::print("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
    fmt::print("  scooterouter\n");
    fmt::print("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
    fmt::print("  map:      {}\n", map_path);
    fmt::print("  scooters: {} (seed: {})\n", n_scooters, actual_seed);
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

    fmt::print("[2/3] Generating fleet...\n");
    sr::FleetManager fleet;
    if (!fleet_path.empty()) {
        fleet.loadFromCSV(fleet_path);
        fmt::print("      loaded from CSV\n");
    } else {
        fleet.generateRandom(graph, n_scooters, actual_seed);
    }
    fmt::print("      {} scooters ({} collectible, {} critical)\n\n",
               fleet.all().size(), fleet.collectible().size(), fleet.critical().size());

    // random van spawn within map margins
    std::mt19937 rng(actual_seed + 1);
    sr::LatLon min_b = graph.minBounds();
    sr::LatLon max_b = graph.maxBounds();
    double lat_margin = (max_b.lat - min_b.lat) * 0.1;
    double lon_margin = (max_b.lon - min_b.lon) * 0.1;

    std::vector<sr::NodeId> valid_nodes;
    for (const auto& [id, node] : graph.allNodes()) {
        if (!graph.neighbors(id).empty() &&
            node.geo.lat > min_b.lat + lat_margin &&
            node.geo.lat < max_b.lat - lat_margin &&
            node.geo.lon > min_b.lon + lon_margin &&
            node.geo.lon < max_b.lon - lon_margin)
            valid_nodes.push_back(id);
    }
    std::uniform_int_distribution<size_t> dist(0, valid_nodes.size() - 1);
    sr::NodeId van_node = valid_nodes[dist(rng)];
    sr::LatLon van_pos  = graph.getNode(van_node).geo;
    fmt::print("      van spawn: ({:.4f}, {:.4f})\n", van_pos.lat, van_pos.lon);

    // fixed warehouse — right of Hamburg center
    sr::LatLon warehouse_pos  = { 53.5580, 10.0150 };
    sr::NodeId warehouse_node = graph.nearestNode(warehouse_pos);

    // build optimized route: van → scooters → warehouse
    fmt::print("[test] Building optimized route...\n");
    sr::PriorityScorer scorer;
    auto ranked   = scorer.ranked(fleet.collectible(), van_pos);
    sr::RouteOptimizer optimizer(nullptr);
    auto optimized = optimizer.optimize(ranked, van_pos, graph);

    sr::AStar astar(sr::Heuristic::euclidean());
    sr::NodeId current = van_node;
    std::vector<sr::NodeId> full_path;
    double total_cost = 0.0;

    for (const auto& s : optimized) {
        sr::NodeId target = graph.nearestNode(s.geo);
        auto result = astar.plan(graph, current, target);

        // if no path found, try neighbors of target node
        if (result.path.empty()) {
            for (const auto& edge : graph.neighbors(target)) {
                result = astar.plan(graph, current, edge.to);
                if (!result.path.empty()) {
                    target = edge.to;
                    break;
                }
            }
        }

        if (!result.path.empty()) {
            full_path.insert(full_path.end(), result.path.begin(), result.path.end());
            total_cost += result.cost;
            current = target;
        } else {
            fmt::print("      WARNING: no path to scooter {} — skipping\n", s.id);
        }
    }

    // return to warehouse
    auto home = astar.plan(graph, current, warehouse_node);
    if (!home.path.empty()) {
        full_path.insert(full_path.end(), home.path.begin(), home.path.end());
        total_cost += home.cost;
    }

    fmt::print("      route: {} nodes, {:.2f} km\n\n", full_path.size(), total_cost);

    fmt::print("[3/3] Initializing renderer...\n");
    sr::Renderer renderer(width, height);
    if (!renderer.init()) {
        fmt::print("Failed to initialize renderer\n");
        return 1;
    }

    sr::MissionController mission(fleet, optimizer, graph);
    renderer.run(mission, graph, fleet, full_path, van_pos, warehouse_pos);

    return 0;
}