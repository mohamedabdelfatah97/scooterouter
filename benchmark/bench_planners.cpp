#include <benchmark/benchmark.h>
#include "core/Graph.h"
#include "map/OSMLoader.h"
#include "map/CostMap.h"
#include "planner/AStar.h"
#include "planner/Dijkstra.h"
#include "planner/BFS.h"
#include "planner/Heuristic.h"

static sr::Graph g_graph;
static std::vector<sr::NodeId> g_nodes;
static bool g_loaded = false;

static void loadGraph() {
    if (g_loaded) return;
    sr::OSMLoader loader;
    loader.load("data/maps/hamburg.osm", g_graph);
    for (const auto& [id, node] : g_graph.allNodes())
        g_nodes.push_back(id);
    g_loaded = true;
}

static void BM_AStar(benchmark::State& state) {
    loadGraph();
    sr::AStar astar(sr::Heuristic::euclidean());
    sr::NodeId n1 = g_nodes[0];
    sr::NodeId n2 = g_nodes[1000];
    for (auto _ : state) {
        auto result = astar.plan(g_graph, n1, n2);
        benchmark::DoNotOptimize(result);
    }
}

static void BM_Dijkstra(benchmark::State& state) {
    loadGraph();
    sr::Dijkstra dijkstra;
    sr::NodeId n1 = g_nodes[0];
    sr::NodeId n2 = g_nodes[1000];
    for (auto _ : state) {
        auto result = dijkstra.plan(g_graph, n1, n2);
        benchmark::DoNotOptimize(result);
    }
}

static void BM_BFS(benchmark::State& state) {
    loadGraph();
    sr::BFS bfs;
    sr::NodeId n1 = g_nodes[0];
    sr::NodeId n2 = g_nodes[1000];
    for (auto _ : state) {
        auto result = bfs.plan(g_graph, n1, n2);
        benchmark::DoNotOptimize(result);
    }
}

BENCHMARK(BM_AStar)->Unit(benchmark::kMillisecond);
BENCHMARK(BM_Dijkstra)->Unit(benchmark::kMillisecond);
BENCHMARK(BM_BFS)->Unit(benchmark::kMillisecond);

BENCHMARK_MAIN();