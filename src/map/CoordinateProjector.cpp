#include "map/CoordinateProjector.h"
#include <cmath>

namespace sr {

void CoordinateProjector::setup(LatLon min_bounds, LatLon max_bounds,
                                 int screen_w, int screen_h, int map_panel_w) {
    min_        = min_bounds;
    max_        = max_bounds;
    screen_w_   = screen_w;
    screen_h_   = screen_h;
    panel_w_    = map_panel_w;
}

Vec2 CoordinateProjector::project(const LatLon& geo) const {
    constexpr float PADDING = 40.0f;

    float usable_w = static_cast<float>(panel_w_)  - PADDING * 2.0f;
    float usable_h = static_cast<float>(screen_h_) - PADDING * 2.0f;

    // Mercator projection: lat uses log(tan) for correct proportions
    auto mercY = [](double lat) {
        double rad = lat * M_PI / 180.0;
        return std::log(std::tan(M_PI / 4.0 + rad / 2.0));
    };

    double y_min = mercY(min_.lat);
    double y_max = mercY(max_.lat);
    double x_min = min_.lon;
    double x_max = max_.lon;

    float x = PADDING + static_cast<float>(
        (geo.lon - x_min) / (x_max - x_min)) * usable_w;

    float y = PADDING + static_cast<float>(
        (y_max - mercY(geo.lat)) / (y_max - y_min)) * usable_h;

    return { x, y };
}

}