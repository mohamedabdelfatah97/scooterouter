#include "planner/DStarLite.h"
#include <algorithm>
#include <cmath>

namespace sr {

static double heuristic(const Node& a, const Node& b) {
    double dlat = a.geo.lat - b.geo.lat;
    double dlon = a.geo.lon - b.geo.lon;
    return std::sqrt(dlat*dlat + dlon*dlon) * 111.0;
}

void DStarLite::initialize(Graph& graph, NodeId start, NodeId goal) {
    graph_  = &graph;
    start_  = start;
    goal_   = goal;
    k_m_    = 0.0;
    states_.clear();
    replan_trace_.clear();

    // clear open list
    while (!open_list_.empty()) open_list_.pop();

    // goal rhs = 0, everything else infinity
    states_[goal_].rhs = 0.0;
    states_[goal_].g   = INFINITY_COST;

    const Node& goal_node  = graph.getNode(goal_);
    const Node& start_node = graph.getNode(start_);
    double h = heuristic(start_node, goal_node);
    open_list_.push({ { h, 0.0 }, goal_ });

    computeShortestPath();
}

DStarLite::Key DStarLite::calculateKey(NodeId u) const {
    const Node& u_node     = graph_->getNode(u);
    const Node& start_node = graph_->getNode(start_);
    double h = heuristic(start_node, u_node);

    auto it = states_.find(u);
    Cost g   = (it != states_.end()) ? it->second.g   : INFINITY_COST;
    Cost rhs = (it != states_.end()) ? it->second.rhs : INFINITY_COST;
    Cost min_g_rhs = std::min(g, rhs);

    return { min_g_rhs + h + k_m_, min_g_rhs };
}

void DStarLite::updateNode(NodeId u) {
    if (u == goal_) {
        states_[u].g = INFINITY_COST;
    }

    // remove u from open list (lazy deletion — mark as outdated)
    // re-insert with new key if locally inconsistent
    auto& s = states_[u];
    if (std::abs(s.g - s.rhs) > 1e-9) {
        open_list_.push({ calculateKey(u), u });
    }
}

void DStarLite::computeShortestPath() {
    while (!open_list_.empty()) {
        auto [k_old, u] = open_list_.top();

        auto it = states_.find(u);
        Cost g   = (it != states_.end()) ? it->second.g   : INFINITY_COST;
        Cost rhs = (it != states_.end()) ? it->second.rhs : INFINITY_COST;

        Key k_new = calculateKey(u);

        if (k_old < k_new) {
            open_list_.pop();
            open_list_.push({ k_new, u });
            continue;
        }

        if (calculateKey(start_) <= k_old && 
            std::abs(states_[start_].g - states_[start_].rhs) < 1e-9) {
            open_list_.pop();
            break;
        }

        open_list_.pop();
        replan_trace_.push_back(u);

        if (g > rhs) {
            states_[u].g = rhs;
            for (const auto& edge : graph_->neighbors(u)) {
                NodeId s = edge.to;
                if (s == goal_) continue;
                Cost new_rhs = edge.cost + states_[u].g;
                if (new_rhs < states_[s].rhs) {
                    states_[s].rhs = new_rhs;
                    updateNode(s);
                }
            }
        } else {
            Cost g_old = g;
            states_[u].g = INFINITY_COST;
            updateNode(u);
            for (const auto& edge : graph_->neighbors(u)) {
                NodeId s = edge.to;
                if (states_[s].rhs == edge.cost + g_old) {
                    if (s != goal_) {
                        states_[s].rhs = INFINITY_COST;
                        for (const auto& e2 : graph_->neighbors(s)) {
                            Cost alt = e2.cost + states_[e2.to].g;
                            states_[s].rhs = std::min(states_[s].rhs, alt);
                        }
                    }
                    updateNode(s);
                }
            }
        }
    }
}

void DStarLite::updateEdge(NodeId u, NodeId v, Cost new_cost) {
    replan_trace_.clear();
    graph_->updateEdgeCost(u, v, new_cost);
    k_m_ += heuristic(graph_->getNode(start_), graph_->getNode(start_));
    updateNode(u);
    computeShortestPath();
}

void DStarLite::updateStart(NodeId new_start) {
    k_m_ += heuristic(graph_->getNode(start_), graph_->getNode(new_start));
    start_ = new_start;
    computeShortestPath();
}

std::vector<NodeId> DStarLite::extractPath() const {
    if (!graph_ || start_ == INVALID_NODE || goal_ == INVALID_NODE)
        return {};

    std::vector<NodeId> path;
    NodeId current = start_;

    for (int steps = 0; steps < 10000; ++steps) {
        path.push_back(current);
        if (current == goal_) break;

        Cost best_cost = INFINITY_COST;
        NodeId best_next = INVALID_NODE;

        for (const auto& edge : graph_->neighbors(current)) {
            auto it = states_.find(edge.to);
            Cost g = (it != states_.end()) ? it->second.g : INFINITY_COST;
            Cost total = edge.cost + g;
            if (total < best_cost) {
                best_cost = total;
                best_next = edge.to;
            }
        }

        if (best_next == INVALID_NODE || best_next == current) break;
        current = best_next;
    }

    return path;
}

Cost DStarLite::totalCost() const {
    auto it = states_.find(start_);
    return (it != states_.end()) ? it->second.g : INFINITY_COST;
}

const std::vector<NodeId>& DStarLite::lastReplannedNodes() const {
    return replan_trace_;
}

} // namespace sr