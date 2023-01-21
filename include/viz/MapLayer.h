#pragma once
#include "../core/Graph.h"
#include "../map/CoordinateProjector.h"
#include <SDL2/SDL.h>

namespace sr {

class MapLayer {
public:
    void draw(SDL_Renderer* r, const Graph& graph,
              const CoordinateProjector& proj);
};

} // namespace sr
