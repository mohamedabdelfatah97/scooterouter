#include "viz/ScooterLayer.h"
#include <optional>

namespace sr {

static void drawCircle(SDL_Renderer* r, int cx, int cy, int radius) {
    for (int w = 0; w < radius * 2; w++) {
        for (int h = 0; h < radius * 2; h++) {
            int dx = radius - w;
            int dy = radius - h;
            if ((dx*dx + dy*dy) <= (radius * radius))
                SDL_RenderDrawPoint(r, cx + dx, cy + dy);
        }
    }
}

void ScooterLayer::draw(SDL_Renderer* r, const FleetManager& fleet,
                        const CoordinateProjector& proj) {
    for (const auto& s : fleet.collectible()) {
        Vec2 pos = proj.project(s.geo);
        int cx = static_cast<int>(pos.x);
        int cy = static_cast<int>(pos.y);

        // dark outline
        SDL_SetRenderDrawColor(r, 0, 0, 0, 255);
        drawCircle(r, cx, cy, 9);

        // white ring
        SDL_SetRenderDrawColor(r, 255, 255, 255, 255);
        drawCircle(r, cx, cy, 8);

        // colored fill by urgency
        if (s.battery_pct < 15)
            SDL_SetRenderDrawColor(r, 255, 50,  50,  255);
        else if (s.battery_pct < 30)
            SDL_SetRenderDrawColor(r, 255, 165, 0,   255);
        else if (s.battery_pct < 60)
            SDL_SetRenderDrawColor(r, 255, 255, 0,   255);
        else
            SDL_SetRenderDrawColor(r, 50,  205, 50,  255);

        drawCircle(r, cx, cy, 6);
    }
}

std::optional<int> ScooterLayer::hitTest(int mouse_x, int mouse_y,
                                          const FleetManager& fleet,
                                          const CoordinateProjector& proj) {
    for (const auto& s : fleet.collectible()) {
        Vec2 pos = proj.project(s.geo);
        int dx = mouse_x - static_cast<int>(pos.x);
        int dy = mouse_y - static_cast<int>(pos.y);
        if (dx*dx + dy*dy <= 9*9)
            return s.id;
    }
    return std::nullopt;
}

} // namespace sr