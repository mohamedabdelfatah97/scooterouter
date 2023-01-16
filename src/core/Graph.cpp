#include "core/Graph.h"
#include <stdexcept>
#include <algorithm>

namespace sr {

void Graph::addNode(const Node& node) {
    nodes_[node.id] = node;
    if (adjacency_.find(node.id) == adjacency_.end()) {
        adjacency_[node.id] = {};
    }
}

void Graph::addEdge(const Edge& edge) {
    adjacency_[edge.from].push_back(edge);
    edge_count_++;
}

const Node& Graph::getNode(NodeId id) const {
    auto it = nodes_.find(id);
    if (it == nodes_.end()) {
        throw std::out_of_range("Node not found");
    }
    return it->second;
}

const std::vector<Edge>& Graph::neighbors(NodeId id) const {
    auto it = adjacency_.find(id);
    if (it == adjacency_.end()) {
        static const std::vector<Edge> empty;
        return empty;
    }
    return it->second;
}

bool Graph::hasNode(NodeId id) const {
    return nodes_.find(id) != nodes_.end();
}

size_t Graph::nodeCount() const { return nodes_.size(); }
size_t Graph::edgeCount() const { return edge_count_; }

LatLon Graph::minBounds() const {
    LatLon min { 90.0, 180.0 };
    for (const auto& [id, node] : nodes_) {
        min.lat = std::min(min.lat, node.geo.lat);
        min.lon = std::min(min.lon, node.geo.lon);
    }
    return min;
}

LatLon Graph::maxBounds() const {
    LatLon max { -90.0, -180.0 };
    for (const auto& [id, node] : nodes_) {
        max.lat = std::max(max.lat, node.geo.lat);
        max.lon = std::max(max.lon, node.geo.lon);
    }
    return max;
}

void Graph::updateEdgeCost(NodeId from, NodeId to, Cost new_cost) {
    auto it = adjacency_.find(from);
    if (it == adjacency_.end()) return;
    for (auto& edge : it->second) {
        if (edge.to == to) { edge.cost = new_cost; return; }
    }
}

const std::unordered_map<NodeId, Node>& Graph::allNodes() const {
    return nodes_;
}

} // namespace sr
