#include "map/CostMap.h"
#include <unordered_map>

namespace sr {

double CostMap::multiplierForHighwayTag(const std::string& tag) {
    // Lower = faster/preferred. Based on typical van driving priorities.
    static const std::unordered_map<std::string, double> table = {
        { "motorway",       1.0 },
        { "motorway_link",  1.1 },
        { "trunk",          1.1 },
        { "trunk_link",     1.2 },
        { "primary",        1.2 },
        { "primary_link",   1.3 },
        { "secondary",      1.4 },
        { "secondary_link", 1.5 },
        { "tertiary",       1.6 },
        { "tertiary_link",  1.7 },
        { "residential",    2.0 },
        { "living_street",  2.5 },
        { "service",        2.5 },
        { "unclassified",   2.0 },
        { "road",           2.0 },
    };
    auto it = table.find(tag);
    return (it != table.end()) ? it->second : 3.0;
}

} // namespace sr
