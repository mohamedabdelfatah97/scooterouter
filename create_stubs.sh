#!/bin/bash
cd ~/dev-cpp/scooterouter

# ── include/core ─────────────────────────────────────────────────────────────
cat > include/core/Types.h << 'EOF'
#pragma once
#include <cstdint>
#include <limits>

namespace sr {

using NodeId = uint64_t;
using EdgeId = uint64_t;
using Cost   = double;

constexpr NodeId INVALID_NODE = std::numeric_limits<NodeId>::max();
constexpr Cost   INFINITY_COST = std::numeric_limits<Cost>::infinity();

struct LatLon {
    double lat = 0.0;
    double lon = 0.0;
};

struct Vec2 {
    float x = 0.0f;
    float y = 0.0f;
};

} // namespace sr
EOF

cat > include/core/Node.h << 'EOF'
#pragma once
#include "Types.h"
#include <string>

namespace sr {

struct Node {
    NodeId  id       = INVALID_NODE;
    LatLon  geo      {};          // WGS84 geographic coords from OSM
    Vec2    screen   {};          // projected pixel coords for SDL2 rendering
};

} // namespace sr
EOF

cat > include/core/Edge.h << 'EOF'
#pragma once
#include "Types.h"
#include <string>

namespace sr {

struct Edge {
    EdgeId  id       = 0;
    NodeId  from     = INVALID_NODE;
    NodeId  to       = INVALID_NODE;
    Cost    cost     = 0.0;       // distance * road_type_multiplier
    bool    one_way  = false;     // parsed from OSM oneway tag
};

} // namespace sr
EOF

cat > include/core/Graph.h << 'EOF'
#pragma once
#include "Node.h"
#include "Edge.h"
#include <unordered_map>
#include <vector>
#include <optional>

namespace sr {

class Graph {
public:
    void addNode(const Node& node);
    void addEdge(const Edge& edge);

    const Node&              getNode(NodeId id) const;
    const std::vector<Edge>& neighbors(NodeId id) const;

    bool hasNode(NodeId id) const;
    size_t nodeCount() const;
    size_t edgeCount() const;

    // Used by CoordinateProjector to compute bounding box
    LatLon minBounds() const;
    LatLon maxBounds() const;

    // Used by DStarLite to update edge cost at runtime
    void updateEdgeCost(NodeId from, NodeId to, Cost new_cost);

    const std::unordered_map<NodeId, Node>& allNodes() const;

private:
    std::unordered_map<NodeId, Node>              nodes_;
    std::unordered_map<NodeId, std::vector<Edge>> adjacency_;
    size_t edge_count_ = 0;
};

} // namespace sr
EOF

# ── include/planner ───────────────────────────────────────────────────────────
cat > include/planner/PlannerInterface.h << 'EOF'
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
EOF

cat > include/planner/Heuristic.h << 'EOF'
#pragma once
#include "../core/Types.h"
#include "../core/Node.h"
#include <cmath>
#include <functional>

namespace sr {

// Heuristic function type: takes two nodes, returns estimated cost
using HeuristicFn = std::function<Cost(const Node&, const Node&)>;

struct Heuristic {
    // Straight-line geographic distance (default for road graphs)
    static HeuristicFn euclidean();

    // Manhattan distance on projected screen coords
    static HeuristicFn manhattan();

    // Octile distance — good for 8-connected grids
    static HeuristicFn octile();

    // Zero heuristic — turns A* into Dijkstra (used for benchmarking)
    static HeuristicFn zero();

    // Factory: select by name from CLI arg --heuristic
    static HeuristicFn fromString(const std::string& name);
};

} // namespace sr
EOF

cat > include/planner/AStar.h << 'EOF'
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
EOF

cat > include/planner/Dijkstra.h << 'EOF'
#pragma once
#include "PlannerInterface.h"

namespace sr {

class Dijkstra : public PlannerInterface {
public:
    PlanResult plan(const Graph& graph, NodeId start, NodeId goal) override;
    std::string name() const override { return "Dijkstra"; }
};

} // namespace sr
EOF

cat > include/planner/BFS.h << 'EOF'
#pragma once
#include "PlannerInterface.h"

namespace sr {

class BFS : public PlannerInterface {
public:
    PlanResult plan(const Graph& graph, NodeId start, NodeId goal) override;
    std::string name() const override { return "BFS"; }
};

} // namespace sr
EOF

cat > include/planner/DStarLite.h << 'EOF'
#pragma once
#include "../core/Graph.h"
#include <unordered_map>
#include <queue>
#include <vector>
#include <utility>

namespace sr {

// D* Lite — Koenig & Likhachev 2002
// Searches backward from goal to start.
// Maintains g and rhs values per node.
// updateEdge() triggers local replan without full restart.
class DStarLite {
public:
    void initialize(const Graph& graph, NodeId start, NodeId goal);

    // Returns current best path from start to goal
    std::vector<NodeId> extractPath() const;

    // Call when an edge cost changes (obstacle added, scooter removed)
    // Triggers local replan — only affected nodes are reprocessed
    void updateEdge(NodeId u, NodeId v, Cost new_cost);

    // Returns nodes touched in last replan — used by FrontierLayer
    const std::vector<NodeId>& lastReplannedNodes() const;

    // Move operator position (called each step of animation)
    void updateStart(NodeId new_start);

    Cost totalCost() const;

private:
    using Key = std::pair<Cost, Cost>;

    struct NodeState {
        Cost g   = INFINITY_COST;
        Cost rhs = INFINITY_COST;
    };

    Key calculateKey(NodeId u) const;
    void updateNode(NodeId u);
    void computeShortestPath();

    const Graph*   graph_  = nullptr;
    NodeId         start_  = INVALID_NODE;
    NodeId         goal_   = INVALID_NODE;
    Cost           k_m_    = 0.0;      // key modifier for start moves

    std::unordered_map<NodeId, NodeState> states_;
    std::priority_queue<
        std::pair<Key, NodeId>,
        std::vector<std::pair<Key, NodeId>>,
        std::greater<>> open_list_;

    std::vector<NodeId> replan_trace_;
};

} // namespace sr
EOF

# ── include/routing ───────────────────────────────────────────────────────────
cat > include/routing/Scooter.h << 'EOF'
#pragma once
#include "../core/Types.h"
#include <string>

namespace sr {

enum class ScooterStatus { Available, Damaged, Missing, Collected };

struct Scooter {
    int           id          = 0;
    LatLon        geo         {};
    int           battery_pct = 0;
    ScooterStatus status      = ScooterStatus::Available;
    NodeId        nearest_node = INVALID_NODE; // snapped to road graph

    bool isCritical()  const { return battery_pct < 15; }
    bool isCollectible() const {
        return status == ScooterStatus::Available ||
               status == ScooterStatus::Damaged;
    }
};

} // namespace sr
EOF

cat > include/routing/PriorityScorer.h << 'EOF'
#pragma once
#include "Scooter.h"
#include "../core/Types.h"
#include <vector>

namespace sr {

// Computes urgency score for each scooter.
// Higher score = collect sooner.
// Score = battery_weight * (100 - battery_pct) + status_weight + distance_weight
class PriorityScorer {
public:
    struct Weights {
        double battery  = 0.6;   // low battery is most urgent
        double status   = 0.3;   // damaged matters regardless of battery
        double distance = 0.1;   // slight bias toward nearby scooters
    };

    explicit PriorityScorer(Weights w = {});

    double score(const Scooter& s, double distance_km) const;

    // Returns scooters sorted by descending urgency score
    std::vector<Scooter> ranked(
        const std::vector<Scooter>& fleet,
        const LatLon& operator_pos) const;

private:
    Weights weights_;
};

} // namespace sr
EOF

cat > include/routing/FleetManager.h << 'EOF'
#pragma once
#include "Scooter.h"
#include <vector>
#include <string>
#include <functional>

namespace sr {

class FleetManager {
public:
    // Load fleet from CSV: id,lat,lon,battery_pct,status
    bool loadFromCSV(const std::string& path);

    const std::vector<Scooter>& all()         const;
    std::vector<Scooter>        collectible()  const;
    std::vector<Scooter>        critical()     const;  // battery < 15%

    // Runtime mutations — called by MissionController on operator events
    void markCollected(int id);
    void markNotFound(int id);

    size_t remaining() const;

    // Callback invoked whenever fleet state changes
    using ChangeCallback = std::function<void()>;
    void onFleetChanged(ChangeCallback cb);

private:
    std::vector<Scooter> fleet_;
    ChangeCallback on_change_;
};

} // namespace sr
EOF

cat > include/routing/RouteOptimizer.h << 'EOF'
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
EOF

cat > include/routing/MissionController.h << 'EOF'
#pragma once
#include "FleetManager.h"
#include "RouteOptimizer.h"
#include "../planner/DStarLite.h"
#include "../core/Graph.h"
#include <memory>
#include <functional>

namespace sr {

enum class MissionState { Idle, Routing, Replanning, Complete };

class MissionController {
public:
    MissionController(
        FleetManager& fleet,
        RouteOptimizer& optimizer,
        const Graph& graph);

    void start(const LatLon& operator_start);
    void update();   // called each frame from Renderer

    // Operator events from SDL2 UI clicks
    void markCollected(int scooter_id);
    void markNotFound(int scooter_id);
    void addObstacle(NodeId u, NodeId v);

    // State queries for Renderer layers
    MissionState              state()        const;
    const std::vector<NodeId>& currentPath() const;
    const std::vector<Scooter>& currentRoute() const;
    const Scooter*             nextTarget()  const;
    LatLon                     operatorPos() const;
    int                        replanCount() const;

    // Callbacks for Renderer to react to replan events
    using ReplanCallback = std::function<void(const std::vector<NodeId>&)>;
    void onReplan(ReplanCallback cb);

private:
    void triggerReplan();
    void advanceToNextScooter();

    FleetManager&    fleet_;
    RouteOptimizer&  optimizer_;
    const Graph&     graph_;
    DStarLite        dstar_;

    std::vector<Scooter> current_route_;
    std::vector<NodeId>  current_path_;
    LatLon               operator_pos_  {};
    MissionState         state_         = MissionState::Idle;
    int                  replan_count_  = 0;
    ReplanCallback       on_replan_;
};

} // namespace sr
EOF

# ── include/map ───────────────────────────────────────────────────────────────
cat > include/map/OSMLoader.h << 'EOF'
#pragma once
#include "../core/Graph.h"
#include <string>

namespace sr {

class OSMLoader {
public:
    // Parse .osm XML and populate graph.
    // Extracts highway nodes + ways, respects oneway tags.
    // Returns false if file not found or parse error.
    bool load(const std::string& osm_path, Graph& graph);

    // Bounding box of loaded map (for CoordinateProjector)
    LatLon minBounds() const;
    LatLon maxBounds() const;

private:
    LatLon min_bounds_ {};
    LatLon max_bounds_ {};
};

} // namespace sr
EOF

cat > include/map/CostMap.h << 'EOF'
#pragma once
#include <string>

namespace sr {

// Maps OSM highway tag values to cost multipliers.
// Applied on top of Haversine distance to get final edge cost.
class CostMap {
public:
    static double multiplierForHighwayTag(const std::string& tag);
};

} // namespace sr
EOF

cat > include/map/CoordinateProjector.h << 'EOF'
#pragma once
#include "../core/Types.h"

namespace sr {

// Converts WGS84 lat/lon to SDL2 screen pixel coordinates.
// Uses Mercator projection scaled to window dimensions.
class CoordinateProjector {
public:
    void setup(LatLon min_bounds, LatLon max_bounds,
               int screen_w, int screen_h, int map_panel_w);

    Vec2 project(const LatLon& geo) const;

private:
    LatLon min_ {}, max_ {};
    int screen_w_ = 0, screen_h_ = 0, panel_w_ = 0;
};

} // namespace sr
EOF

cat > include/map/ObstacleInflation.h << 'EOF'
#pragma once
#include "../core/Graph.h"

namespace sr {

// Applies cost inflation around blocked edges.
// Radius controls how many hops out from the obstacle to inflate.
// Called after operator adds an obstacle at runtime.
class ObstacleInflation {
public:
    explicit ObstacleInflation(int radius = 2, double inflation_factor = 3.0);
    void inflate(Graph& graph, NodeId blocked_from, NodeId blocked_to);

private:
    int    radius_;
    double factor_;
};

} // namespace sr
EOF

# ── include/viz ───────────────────────────────────────────────────────────────
cat > include/viz/Renderer.h << 'EOF'
#pragma once
#include "../routing/MissionController.h"
#include "../core/Graph.h"
#include "MapLayer.h"
#include "ScooterLayer.h"
#include "PathLayer.h"
#include "FrontierLayer.h"
#include "UIOverlay.h"
#include <SDL2/SDL.h>
#include <memory>

namespace sr {

class Renderer {
public:
    Renderer(int width, int height);
    ~Renderer();

    bool init();
    void run(MissionController& mission, const Graph& graph,
             const FleetManager& fleet);

private:
    void handleEvents(MissionController& mission);
    void render(const MissionController& mission,
                const Graph& graph,
                const FleetManager& fleet);

    int width_, height_;
    SDL_Window*   window_   = nullptr;
    SDL_Renderer* renderer_ = nullptr;

    MapLayer      map_layer_;
    ScooterLayer  scooter_layer_;
    PathLayer     path_layer_;
    FrontierLayer frontier_layer_;
    UIOverlay     ui_overlay_;
};

} // namespace sr
EOF

cat > include/viz/MapLayer.h << 'EOF'
#pragma once
#include "../core/Graph.h"
#include "../map/CoordinateProjector.h"
#include <SDL2/SDL.h>

namespace sr {

class MapLayer {
public:
    void draw(SDL_Renderer* r, const Graph& graph,
              const CoordinateProjector& proj);
};

} // namespace sr
EOF

cat > include/viz/ScooterLayer.h << 'EOF'
#pragma once
#include "../routing/FleetManager.h"
#include "../map/CoordinateProjector.h"
#include <SDL2/SDL.h>
#include <optional>

namespace sr {

class ScooterLayer {
public:
    void draw(SDL_Renderer* r, const FleetManager& fleet,
              const CoordinateProjector& proj);

    // Returns scooter id if click hit a marker, nullopt otherwise
    std::optional<int> hitTest(int mouse_x, int mouse_y,
                               const FleetManager& fleet,
                               const CoordinateProjector& proj);
};

} // namespace sr
EOF

cat > include/viz/PathLayer.h << 'EOF'
#pragma once
#include "../core/Graph.h"
#include "../map/CoordinateProjector.h"
#include <SDL2/SDL.h>
#include <vector>

namespace sr {

class PathLayer {
public:
    void draw(SDL_Renderer* r,
              const std::vector<NodeId>& path,
              const Graph& graph,
              const CoordinateProjector& proj,
              size_t visited_up_to);   // nodes before this index drawn gray
};

} // namespace sr
EOF

cat > include/viz/FrontierLayer.h << 'EOF'
#pragma once
#include "../core/Graph.h"
#include "../map/CoordinateProjector.h"
#include <SDL2/SDL.h>
#include <vector>
#include <chrono>

namespace sr {

// Flashes replanned nodes in orange, fades over ~800ms
class FrontierLayer {
public:
    void triggerReplan(const std::vector<NodeId>& nodes);
    void draw(SDL_Renderer* r, const Graph& graph,
              const CoordinateProjector& proj);
    bool isActive() const;

private:
    std::vector<NodeId> nodes_;
    std::chrono::steady_clock::time_point triggered_at_;
    bool active_ = false;
    static constexpr int FADE_MS = 800;
};

} // namespace sr
EOF

cat > include/viz/UIOverlay.h << 'EOF'
#pragma once
#include "../routing/MissionController.h"
#include "../routing/FleetManager.h"
#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>

namespace sr {

class UIOverlay {
public:
    bool init(SDL_Renderer* r);
    void draw(SDL_Renderer* r,
              const MissionController& mission,
              const FleetManager& fleet,
              int panel_x, int panel_w, int screen_h);
    ~UIOverlay();

private:
    TTF_Font* font_       = nullptr;
    TTF_Font* font_small_ = nullptr;
};

} // namespace sr
EOF

# ── src stubs ─────────────────────────────────────────────────────────────────
for f in core/Node core/Edge core/Graph \
          planner/AStar planner/Dijkstra planner/BFS planner/DStarLite \
          routing/Scooter routing/FleetManager routing/PriorityScorer \
          routing/RouteOptimizer routing/MissionController \
          map/OSMLoader map/CostMap map/ObstacleInflation map/CoordinateProjector \
          viz/Renderer viz/MapLayer viz/ScooterLayer viz/PathLayer \
          viz/FrontierLayer viz/UIOverlay; do
    module=$(basename $f)
    dir=$(dirname $f)
    cat > src/${f}.cpp << CPPEOF
#include "${dir}/${module}.h"

namespace sr {

// TODO: implement ${module}

} // namespace sr
CPPEOF
done

# main.cpp
cat > src/main.cpp << 'EOF'
#include <CLI/CLI.hpp>
#include <fmt/core.h>
#include "core/Graph.h"
#include "map/OSMLoader.h"
#include "routing/FleetManager.h"
#include "routing/RouteOptimizer.h"
#include "routing/MissionController.h"
#include "planner/AStar.h"
#include "viz/Renderer.h"
#include <memory>

int main(int argc, char** argv) {
    CLI::App app{"scooterouter — e-scooter collection route planner"};

    std::string map_path   = "data/maps/munich.osm";
    std::string fleet_path = "data/scooters/fleet.csv";
    std::string heuristic  = "euclidean";
    int width = 1400, height = 800;

    app.add_option("--map",       map_path,   "Path to .osm map file");
    app.add_option("--fleet",     fleet_path, "Path to fleet CSV");
    app.add_option("--heuristic", heuristic,  "Heuristic: euclidean|manhattan|octile");
    app.add_option("--width",     width,      "Window width");
    app.add_option("--height",    height,     "Window height");

    CLI11_PARSE(app, argc, argv);

    // TODO: wire up components as they are implemented
    fmt::print("scooterouter starting...\n");
    fmt::print("map:   {}\n", map_path);
    fmt::print("fleet: {}\n", fleet_path);

    return 0;
}
EOF

# tests
for t in test_astar test_dstar test_scorer test_optimizer test_graph; do
cat > tests/${t}.cpp << TESTEOF
#include <gtest/gtest.h>
// TODO: ${t} tests

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
TESTEOF
done

# benchmark
cat > benchmark/bench_planners.cpp << 'EOF'
#include <benchmark/benchmark.h>
// TODO: benchmark fixtures for A*, Dijkstra, BFS

BENCHMARK_MAIN();
EOF

# scripts
cat > scripts/generate_fleet.py << 'EOF'
#!/usr/bin/env python3
"""
generate_fleet.py
Generates fake fleet.csv with N scooters in a city bounding box.
Usage: python3 scripts/generate_fleet.py --city munich --count 30
"""
import argparse, random, csv, os

CITIES = {
    "munich":  {"lat": (48.12, 48.16), "lon": (11.55, 11.62)},
    "berlin":  {"lat": (52.48, 52.54), "lon": (13.37, 13.45)},
    "hamburg": {"lat": (53.54, 53.58), "lon": (9.97,  10.04)},
}

def main():
    p = argparse.ArgumentParser()
    p.add_argument("--city",   default="munich")
    p.add_argument("--count",  type=int, default=20)
    p.add_argument("--output", default="data/scooters/fleet.csv")
    args = p.parse_args()

    bounds = CITIES.get(args.city, CITIES["munich"])
    rows = []
    for i in range(1, args.count + 1):
        lat = random.uniform(*bounds["lat"])
        lon = random.uniform(*bounds["lon"])
        bat = random.randint(2, 95)
        status = random.choices(
            ["available", "available", "available", "damaged"],
            weights=[70, 70, 70, 20])[0]
        rows.append([i, f"{lat:.4f}", f"{lon:.4f}", bat, status])

    os.makedirs(os.path.dirname(args.output), exist_ok=True)
    with open(args.output, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["id", "lat", "lon", "battery_pct", "status"])
        w.writerows(rows)
    print(f"Generated {args.count} scooters → {args.output}")

if __name__ == "__main__":
    main()
EOF

cat > scripts/download_osm.sh << 'EOF'
#!/bin/bash
# download_osm.sh — download OSM extract via Overpass API
# Usage: ./scripts/download_osm.sh munich
# Output: data/maps/munich.osm

CITY=${1:-munich}
declare -A BBOX
BBOX[munich]="48.12,11.55,48.16,11.62"
BBOX[berlin]="52.48,13.37,52.54,13.45"
BBOX[hamburg]="53.54,9.97,53.58,10.04"

BOX=${BBOX[$CITY]}
if [ -z "$BOX" ]; then
  echo "Unknown city: $CITY. Available: munich, berlin, hamburg"
  exit 1
fi

OUTPUT="data/maps/${CITY}.osm"
URL="https://overpass-api.de/api/map?bbox=${BOX}"

echo "Downloading OSM extract for ${CITY} (bbox: ${BOX})..."
curl -L "$URL" -o "$OUTPUT"
echo "Saved to ${OUTPUT}"
EOF
chmod +x scripts/download_osm.sh

echo ""
echo "✅ All files created. Final structure:"
find . -not -path './.git/*' -not -path './build/*' -not -path './vcpkg/*' \
  | sort | grep -v "^.$"

