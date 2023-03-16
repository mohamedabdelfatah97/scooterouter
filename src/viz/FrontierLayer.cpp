#include "viz/FrontierLayer.h"

namespace sr {

void FrontierLayer::triggerReplan(const std::vector<NodeId>& nodes) {
    nodes_        = nodes;
    triggered_at_ = std::chrono::steady_clock::now();
    active_       = true;
}

void FrontierLayer::draw(SDL_Renderer* r, const Graph& graph,
                         const CoordinateProjector& proj) {
    if (!active_) return;

    auto now     = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                       now - triggered_at_).count();

    if (elapsed > FADE_MS) { active_ = false; return; }

    Uint8 alpha = static_cast<Uint8>(255 * (1.0f - (float)elapsed / FADE_MS));
    SDL_SetRenderDrawColor(r, 255, 140, 0, alpha);

    for (NodeId id : nodes_) {
        if (!graph.hasNode(id)) continue;
        Vec2 pos = proj.project(graph.getNode(id).geo);
        SDL_Rect rect {
            static_cast<int>(pos.x) - 3,
            static_cast<int>(pos.y) - 3,
            6, 6
        };
        SDL_RenderFillRect(r, &rect);
    }
}

bool FrontierLayer::isActive() const { return active_; }

} // namespace sr
