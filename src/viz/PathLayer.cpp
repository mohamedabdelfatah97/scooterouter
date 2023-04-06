#include "viz/PathLayer.h"

namespace sr {

void PathLayer::draw(SDL_Renderer* r,
                     const std::vector<NodeId>& path,
                     const Graph& graph,
                     const CoordinateProjector& proj,
                     size_t visited_up_to,
                     PathColor color) {
    if (path.size() < 2) return;

    for (size_t i = 0; i + 1 < path.size(); ++i) {
        if (!graph.hasNode(path[i]) || !graph.hasNode(path[i+1])) continue;

        Vec2 from = proj.project(graph.getNode(path[i]).geo);
        Vec2 to   = proj.project(graph.getNode(path[i+1]).geo);

        // visited segments gray, upcoming segments in algorithm color
        if (i < visited_up_to)
            SDL_SetRenderDrawColor(r, 80, 80, 80, 255);
        else
            SDL_SetRenderDrawColor(r, color.r, color.g, color.b, 255);

        for (int offset = -1; offset <= 1; ++offset) {
            SDL_RenderDrawLine(r,
                static_cast<int>(from.x) + offset,
                static_cast<int>(from.y),
                static_cast<int>(to.x) + offset,
                static_cast<int>(to.y));
            SDL_RenderDrawLine(r,
                static_cast<int>(from.x),
                static_cast<int>(from.y) + offset,
                static_cast<int>(to.x),
                static_cast<int>(to.y) + offset);
        }
    }
}

} // namespace sr