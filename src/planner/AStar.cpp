#include "planner/AStar.h"
#include <queue>
#include <unordered_map>
#include <algorithm>
#include <cmath>

namespace sr {

AStar::AStar(HeuristicFn heuristic) : heuristic_(std::move(heuristic)) {}

PlanResult AStar::plan(const Graph& graph, NodeId start, NodeId goal) {
    expanded_trace_.clear();

    if (!graph.hasNode(start) || !graph.hasNode(goal))
        return { {}, INFINITY_COST, 0 };

    // min-heap: (f_cost, node_id)
    using Entry = std::pair<Cost, NodeId>;
    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;

    std::unordered_map<NodeId, Cost>   g_cost;
    std::unordered_map<NodeId, NodeId> came_from;
    std::unordered_map<NodeId, bool>   closed;

    g_cost[start] = 0.0;
    const Node& goal_node = graph.getNode(goal);
    Cost h = heuristic_(graph.getNode(start), goal_node);
    open.push({ h, start });

    size_t nodes_expanded = 0;

    while (!open.empty()) {
        auto [f, current] = open.top();
        open.pop();

        if (closed[current]) continue;
        closed[current] = true;
        expanded_trace_.push_back(current);
        nodes_expanded++;

        if (current == goal) {
            // reconstruct path
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
                Cost h_next = heuristic_(graph.getNode(edge.to), goal_node);
                open.push({ tentative_g + h_next, edge.to });
            }
        }
    }

    return { {}, INFINITY_COST, nodes_expanded }; // no path found
}

const std::vector<NodeId>& AStar::lastExpandedNodes() const {
    return expanded_trace_;
}

} // namespace sr