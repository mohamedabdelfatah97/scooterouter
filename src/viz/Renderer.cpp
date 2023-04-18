#include "viz/Renderer.h"
#include "map/CoordinateProjector.h"
#include <fmt/core.h>

namespace sr {

Renderer::Renderer(int width, int height)
    : width_(width), height_(height) {}

Renderer::~Renderer() {
    if (renderer_) SDL_DestroyRenderer(renderer_);
    if (window_)   SDL_DestroyWindow(window_);
    TTF_Quit();
    SDL_Quit();
}

bool Renderer::init() {
    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        fmt::print("[Renderer] SDL_Init failed: {}\n", SDL_GetError());
        return false;
    }

    if (TTF_Init() != 0) {
        fmt::print("[Renderer] TTF_Init failed: {}\n", TTF_GetError());
        return false;
    }

    // SDL_WINDOW_ALWAYS_ON_TOP ensures window appears on macOS from terminal
    window_ = SDL_CreateWindow(
        "scooterouter",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        width_, height_,
        SDL_WINDOW_SHOWN | SDL_WINDOW_ALWAYS_ON_TOP
    );
    if (!window_) {
        fmt::print("[Renderer] CreateWindow failed: {}\n", SDL_GetError());
        return false;
    }

    // Bring window to front — required on macOS to get focus
    SDL_RaiseWindow(window_);

    renderer_ = SDL_CreateRenderer(
        window_, -1,
        SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC
    );
    if (!renderer_) {
        fmt::print("[Renderer] CreateRenderer failed: {}\n", SDL_GetError());
        return false;
    }

    fmt::print("[Renderer] Window {}x{} initialized\n", width_, height_);
    return true;
}

void Renderer::run(MissionController& mission, Graph& graph,
                   const FleetManager& fleet,
                   const std::vector<NodeId>& initial_path) {
    int map_panel_w = static_cast<int>(width_ * 0.75f);

    CoordinateProjector proj;
    proj.setup(graph.minBounds(), graph.maxBounds(),
               width_, height_, map_panel_w);

    // working copy of path — D* Lite can update this
    std::vector<NodeId> path = initial_path;

    bool running = true;
    SDL_Event event;

    SDL_Delay(500);
    SDL_FlushEvents(SDL_FIRSTEVENT, SDL_LASTEVENT);

    while (running) {
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) running = false;
            if (event.type == SDL_KEYDOWN) {
                if (event.key.keysym.sym == SDLK_ESCAPE) running = false;
                if (event.key.keysym.sym == SDLK_r) {
                    fmt::print("[Renderer] R pressed — triggering D* Lite replan\n");
                    replan_requested_ = true;
                }
            }
        }

        // handle replan request
        if (replan_requested_ && path.size() >= 2) {
            replan_requested_ = false;
            NodeId start = path.front();
            NodeId goal  = path.back();
            // block first edge to simulate obstacle
            graph.updateEdgeCost(path[0], path[1], INFINITY_COST);
            dstar_.initialize(graph, start, goal);
            auto new_path = dstar_.extractPath();
            if (!new_path.empty()) {
                path = new_path;
                fmt::print("[Renderer] Replan complete: {} nodes\n", path.size());
            }
        }

        SDL_SetRenderDrawColor(renderer_, 18, 18, 18, 255);
        SDL_RenderClear(renderer_);

        map_layer_.draw(renderer_, graph, proj);
        scooter_layer_.draw(renderer_, fleet, proj);
        path_layer_.draw(renderer_, path, graph, proj, 0);

        SDL_RenderPresent(renderer_);
        SDL_Delay(16);
    }
}

void Renderer::handleEvents(MissionController& mission) {}
void Renderer::render(const MissionController& mission,
                      const Graph& graph,
                      const FleetManager& fleet) {}

} // namespace sr