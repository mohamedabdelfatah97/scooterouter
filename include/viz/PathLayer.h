#pragma once
#include "../core/Graph.h"
#include "../map/CoordinateProjector.h"
#include <SDL2/SDL.h>
#include <vector>

namespace sr {

struct PathColor {
    Uint8 r, g, b;
};

// predefined algorithm colors
static constexpr PathColor PATH_ASTAR    = { 0,   100, 255 };
static constexpr PathColor PATH_DIJKSTRA = { 0,   220, 100 };
static constexpr PathColor PATH_BFS      = { 255, 80,  180 };
static constexpr PathColor PATH_DSTAR    = { 255, 220, 0   };

class PathLayer {
public:
    void draw(SDL_Renderer* r,
              const std::vector<NodeId>& path,
              const Graph& graph,
              const CoordinateProjector& proj,
              size_t visited_up_to,
              PathColor color = PATH_ASTAR);
};

} // namespace sr