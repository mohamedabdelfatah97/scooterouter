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
