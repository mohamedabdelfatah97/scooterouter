#pragma once
#include "../routing/MissionController.h"
#include "../core/Graph.h"
#include "MapLayer.h"
#include "ScooterLayer.h"
#include "PathLayer.h"
#include "FrontierLayer.h"
#include "UIOverlay.h"
#include <SDL2/SDL.h>
#include <memory>
#include <SDL2/SDL.h>
#include <memory>
#include <vector>

namespace sr {

class Renderer {
public:
    Renderer(int width, int height);
    ~Renderer();

    bool init();
    void run(MissionController& mission, const Graph& graph,
         const FleetManager& fleet,
         const std::vector<NodeId>& path = {});

private:
    void handleEvents(MissionController& mission);
    void render(const MissionController& mission,
                const Graph& graph,
                const FleetManager& fleet);

    int width_, height_;
    SDL_Window*   window_   = nullptr;
    SDL_Renderer* renderer_ = nullptr;

    MapLayer      map_layer_;
    ScooterLayer  scooter_layer_;
    PathLayer     path_layer_;
    FrontierLayer frontier_layer_;
    UIOverlay     ui_overlay_;
};

} // namespace sr
