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
