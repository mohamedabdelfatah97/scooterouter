#pragma once
#include "../core/Types.h"

namespace sr {

// Converts WGS84 lat/lon to SDL2 screen pixel coordinates.
// Uses Mercator projection scaled to window dimensions.
class CoordinateProjector {
public:
    void setup(LatLon min_bounds, LatLon max_bounds,
               int screen_w, int screen_h, int map_panel_w);

    Vec2 project(const LatLon& geo) const;

private:
    LatLon min_ {}, max_ {};
    int screen_w_ = 0, screen_h_ = 0, panel_w_ = 0;
};

} // namespace sr
