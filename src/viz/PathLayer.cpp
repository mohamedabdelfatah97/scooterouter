#include "viz/PathLayer.h"

namespace sr {

void PathLayer::draw(SDL_Renderer* r,
                     const std::vector<NodeId>& path,
                     const Graph& graph,
                     const CoordinateProjector& proj,
                     size_t visited_up_to,
                     PathColor color,
                     PathOffset offset) {
    if (path.size() < 2) return;

    for (size_t i = 0; i + 1 < path.size(); ++i) {
        if (!graph.hasNode(path[i]) || !graph.hasNode(path[i+1])) continue;

        Vec2 from = proj.project(graph.getNode(path[i]).geo);
        Vec2 to   = proj.project(graph.getNode(path[i+1]).geo);

        if (i < visited_up_to)
            SDL_SetRenderDrawColor(r, 80, 80, 80, 255);
        else
            SDL_SetRenderDrawColor(r, color.r, color.g, color.b, 255);

        int fx = static_cast<int>(from.x) + offset.dx;
        int fy = static_cast<int>(from.y) + offset.dy;
        int tx = static_cast<int>(to.x)   + offset.dx;
        int ty = static_cast<int>(to.y)   + offset.dy;

        for (int o = -1; o <= 1; ++o) {
            SDL_RenderDrawLine(r, fx+o, fy, tx+o, ty);
            SDL_RenderDrawLine(r, fx, fy+o, tx, ty+o);
        }
    }
}

} // namespace sr