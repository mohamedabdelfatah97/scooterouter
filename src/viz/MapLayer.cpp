#include "viz/MapLayer.h"

namespace sr {

void MapLayer::draw(SDL_Renderer* r, const Graph& graph,
                    const CoordinateProjector& proj) {
    // dark gray roads on dark background
    SDL_SetRenderDrawColor(r, 60, 60, 60, 255);

    for (const auto& [id, node] : graph.allNodes()) {
        Vec2 from = proj.project(node.geo);

        for (const auto& edge : graph.neighbors(id)) {
            if (!graph.hasNode(edge.to)) continue;
            Vec2 to = proj.project(graph.getNode(edge.to).geo);

            SDL_RenderDrawLine(r,
                static_cast<int>(from.x), static_cast<int>(from.y),
                static_cast<int>(to.x),   static_cast<int>(to.y));
        }
    }
}

} // namespace sr