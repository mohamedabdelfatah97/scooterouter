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
    void initialize(Graph& graph, NodeId start, NodeId goal);

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

    Graph*   graph_  = nullptr;
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
