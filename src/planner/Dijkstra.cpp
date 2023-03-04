#include "planner/Dijkstra.h"
#include <queue>
#include <unordered_map>
#include <algorithm>

namespace sr {

PlanResult Dijkstra::plan(const Graph& graph, NodeId start, NodeId goal) {
    if (!graph.hasNode(start) || !graph.hasNode(goal))
        return { {}, INFINITY_COST, 0 };

    using Entry = std::pair<Cost, NodeId>;
    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;

    std::unordered_map<NodeId, Cost>   g_cost;
    std::unordered_map<NodeId, NodeId> came_from;
    std::unordered_map<NodeId, bool>   closed;

    g_cost[start] = 0.0;
    open.push({ 0.0, start });

    size_t nodes_expanded = 0;

    while (!open.empty()) {
        auto [cost, current] = open.top();
        open.pop();

        if (closed[current]) continue;
        closed[current] = true;
        nodes_expanded++;

        if (current == goal) {
            std::vector<NodeId> path;
            NodeId node = goal;
            while (node != start) {
                path.push_back(node);
                node = came_from[node];
            }
            path.push_back(start);
            std::reverse(path.begin(), path.end());
            return { path, g_cost[goal], nodes_expanded };
        }

        for (const auto& edge : graph.neighbors(current)) {
            if (closed[edge.to]) continue;
            Cost tentative_g = g_cost[current] + edge.cost;
            if (g_cost.find(edge.to) == g_cost.end() ||
                tentative_g < g_cost[edge.to]) {
                g_cost[edge.to]    = tentative_g;
                came_from[edge.to] = current;
                open.push({ tentative_g, edge.to });
            }
        }
    }

    return { {}, INFINITY_COST, nodes_expanded };
}

} // namespace sr