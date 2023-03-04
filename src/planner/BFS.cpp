#include "planner/BFS.h"
#include <queue>
#include <unordered_map>
#include <algorithm>

namespace sr {

PlanResult BFS::plan(const Graph& graph, NodeId start, NodeId goal) {
    if (!graph.hasNode(start) || !graph.hasNode(goal))
        return { {}, INFINITY_COST, 0 };

    std::queue<NodeId>                 open;
    std::unordered_map<NodeId, NodeId> came_from;
    std::unordered_map<NodeId, bool>   visited;

    open.push(start);
    visited[start] = true;
    came_from[start] = start;

    size_t nodes_expanded = 0;

    while (!open.empty()) {
        NodeId current = open.front();
        open.pop();
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

            // compute actual cost along path
            Cost total = 0.0;
            for (size_t i = 0; i + 1 < path.size(); ++i) {
                for (const auto& edge : graph.neighbors(path[i])) {
                    if (edge.to == path[i+1]) {
                        total += edge.cost;
                        break;
                    }
                }
            }
            return { path, total, nodes_expanded };
        }

        for (const auto& edge : graph.neighbors(current)) {
            if (!visited[edge.to]) {
                visited[edge.to]    = true;
                came_from[edge.to]  = current;
                open.push(edge.to);
            }
        }
    }

    return { {}, INFINITY_COST, nodes_expanded };
}

} // namespace sr