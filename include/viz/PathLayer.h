#pragma once
#include "../core/Graph.h"
#include "../map/CoordinateProjector.h"
#include <SDL2/SDL.h>
#include <vector>

namespace sr {

class PathLayer {
public:
    void draw(SDL_Renderer* r,
              const std::vector<NodeId>& path,
              const Graph& graph,
              const CoordinateProjector& proj,
              size_t visited_up_to);   // nodes before this index drawn gray
};

} // namespace sr
