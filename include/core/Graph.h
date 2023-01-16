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
