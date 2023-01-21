#pragma once
#include "../routing/FleetManager.h"
#include "../map/CoordinateProjector.h"
#include <SDL2/SDL.h>
#include <optional>

namespace sr {

class ScooterLayer {
public:
    void draw(SDL_Renderer* r, const FleetManager& fleet,
              const CoordinateProjector& proj);

    // Returns scooter id if click hit a marker, nullopt otherwise
    std::optional<int> hitTest(int mouse_x, int mouse_y,
                               const FleetManager& fleet,
                               const CoordinateProjector& proj);
};

} // namespace sr
