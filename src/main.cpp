#include <CLI/CLI.hpp>
#include <fmt/core.h>
#include "core/Graph.h"
#include "map/OSMLoader.h"
#include "map/CoordinateProjector.h"
#include "routing/FleetManager.h"
#include "routing/MissionController.h"
#include "viz/Renderer.h"
#include "routing/RouteOptimizer.h"

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

    // Load road graph from OSM
    fmt::print("[1/3] Loading map...\n");
    sr::Graph graph;
    sr::OSMLoader loader;
    if (!loader.load(map_path, graph)) {
        fmt::print("Failed to load map: {}\n", map_path);
        return 1;
    }

    auto min_b = loader.minBounds();
    auto max_b = loader.maxBounds();
    fmt::print("      Bounds: ({:.4f}, {:.4f}) → ({:.4f}, {:.4f})\n\n",
               min_b.lat, min_b.lon, max_b.lat, max_b.lon);

    // Load scooter fleet
    fmt::print("[2/3] Loading fleet...\n");
    sr::FleetManager fleet;
    if (!fleet.loadFromCSV(fleet_path)) {
        fmt::print("Failed to load fleet: {}\n", fleet_path);
        return 1;
    }
    fmt::print("      {} scooters loaded ({} collectible, {} critical)\n\n",
               fleet.all().size(),
               fleet.collectible().size(),
               fleet.critical().size());

    // Initialize renderer and open window
    fmt::print("[3/3] Initializing renderer...\n");
    sr::Renderer renderer(width, height);
    if (!renderer.init()) {
        fmt::print("Failed to initialize renderer\n");
        return 1;
    }

    sr::RouteOptimizer optimizer(nullptr);
    sr::MissionController mission(fleet, optimizer, graph);

    return 0;
}