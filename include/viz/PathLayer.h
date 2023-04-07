#pragma once
#include "../core/Graph.h"
#include "../map/CoordinateProjector.h"
#include <SDL2/SDL.h>
#include <vector>

namespace sr {

struct PathColor {
    Uint8 r, g, b;
};

static constexpr PathColor PATH_ASTAR    = { 0,   100, 255 };
static constexpr PathColor PATH_DIJKSTRA = { 0,   220, 100 };
static constexpr PathColor PATH_BFS      = { 255, 80,  180 };
static constexpr PathColor PATH_DSTAR    = { 255, 220, 0   };

struct PathOffset {
    int dx, dy;
};

static constexpr PathOffset OFFSET_ASTAR    = {  0,  0 };
static constexpr PathOffset OFFSET_DIJKSTRA = {  3,  0 };
static constexpr PathOffset OFFSET_BFS      = {  0,  3 };
static constexpr PathOffset OFFSET_DSTAR    = {  3,  3 };

class PathLayer {
public:
    void draw(SDL_Renderer* r,
              const std::vector<NodeId>& path,
              const Graph& graph,
              const CoordinateProjector& proj,
              size_t visited_up_to,
              PathColor color = PATH_ASTAR,
              PathOffset offset = OFFSET_ASTAR);
};

} // namespace sr