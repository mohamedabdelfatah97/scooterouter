#pragma once
#include "Types.h"
#include <string>

namespace sr {

struct Node {
    NodeId  id       = INVALID_NODE;
    LatLon  geo      {};          // WGS84 geographic coords from OSM
    Vec2    screen   {};          // projected pixel coords for SDL2 rendering
};

} // namespace sr
