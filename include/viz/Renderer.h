#pragma once
#include "../routing/MissionController.h"
#include "../core/Graph.h"
#include "../core/Types.h"
#include "MapLayer.h"
#include "ScooterLayer.h"
#include "PathLayer.h"
#include "FrontierLayer.h"
#include "UIOverlay.h"
#include "../planner/DStarLite.h"
#include <SDL2/SDL.h>
#include <memory>
#include <vector>

namespace sr {

class Renderer {
public:
    Renderer(int width, int height);
    ~Renderer();

    bool init();
    void run(MissionController& mission, Graph& graph,
             const FleetManager& fleet,
             const std::vector<NodeId>& path = {},
             LatLon van_pos       = {0.0, 0.0},
             LatLon warehouse_pos = {0.0, 0.0});

private:
    void handleEvents(MissionController& mission);
    void render(const MissionController& mission,
                const Graph& graph,
                const FleetManager& fleet);

    int width_, height_;
    int  replan_count_    = 0;
    bool paused_          = false;
    bool replan_requested_ = false;
    bool show_fleet_      = true;
    bool show_path_       = true;

    SDL_Window*   window_   = nullptr;
    SDL_Renderer* renderer_ = nullptr;

    MapLayer      map_layer_;
    ScooterLayer  scooter_layer_;
    PathLayer     path_layer_;
    FrontierLayer frontier_layer_;
    UIOverlay     ui_overlay_;
    DStarLite     dstar_;

    LatLon van_pos_       = {0.0, 0.0};
    LatLon warehouse_pos_ = {0.0, 0.0};
};

} // namespace sr