#pragma once
#include "../core/Graph.h"
#include "../map/CoordinateProjector.h"
#include <SDL2/SDL.h>
#include <vector>
#include <chrono>

namespace sr {

// Flashes replanned nodes in orange, fades over ~800ms
class FrontierLayer {
public:
    void triggerReplan(const std::vector<NodeId>& nodes);
    void draw(SDL_Renderer* r, const Graph& graph,
              const CoordinateProjector& proj);
    bool isActive() const;

private:
    std::vector<NodeId> nodes_;
    std::chrono::steady_clock::time_point triggered_at_;
    bool active_ = false;
    static constexpr int FADE_MS = 800;
};

} // namespace sr
