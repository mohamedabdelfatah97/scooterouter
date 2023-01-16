#pragma once
#include "Types.h"
#include <string>

namespace sr {

struct Edge {
    EdgeId  id       = 0;
    NodeId  from     = INVALID_NODE;
    NodeId  to       = INVALID_NODE;
    Cost    cost     = 0.0;       // distance * road_type_multiplier
    bool    one_way  = false;     // parsed from OSM oneway tag
};

} // namespace sr
