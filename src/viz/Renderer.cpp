#include "viz/Renderer.h"
#include "map/CoordinateProjector.h"
#include "planner/AStar.h"
#include "planner/Heuristic.h"
#include <fmt/core.h>

static void drawVan(SDL_Renderer* r, int x, int y) {
    // simple arrow/triangle pointing up
    SDL_SetRenderDrawColor(r, 255, 255, 255, 255);
    // body
    SDL_RenderDrawLine(r, x-6, y+4, x+6, y+4);
    SDL_RenderDrawLine(r, x-6, y+4, x-6, y-2);
    SDL_RenderDrawLine(r, x+6, y+4, x+6, y-2);
    SDL_RenderDrawLine(r, x-6, y-2, x,   y-8);
    SDL_RenderDrawLine(r, x+6, y-2, x,   y-8);
    // wheels
    SDL_SetRenderDrawColor(r, 180, 180, 180, 255);
    SDL_RenderDrawLine(r, x-5, y+4, x-5, y+7);
    SDL_RenderDrawLine(r, x+5, y+4, x+5, y+7);
}

static void drawWarehouse(SDL_Renderer* r, int x, int y) {
    // house shape: square base + triangle roof
    SDL_SetRenderDrawColor(r, 255, 200, 50, 255);
    // base
    SDL_RenderDrawLine(r, x-8, y+6,  x+8, y+6);
    SDL_RenderDrawLine(r, x-8, y+6,  x-8, y-2);
    SDL_RenderDrawLine(r, x+8, y+6,  x+8, y-2);
    SDL_RenderDrawLine(r, x-8, y-2,  x+8, y-2);
    // roof
    SDL_RenderDrawLine(r, x-8, y-2,  x,   y-10);
    SDL_RenderDrawLine(r, x+8, y-2,  x,   y-10);
    // door
    SDL_RenderDrawLine(r, x-2, y+6,  x-2, y+1);
    SDL_RenderDrawLine(r, x+2, y+6,  x+2, y+1);
    SDL_RenderDrawLine(r, x-2, y+1,  x+2, y+1);
}

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
                   const std::vector<NodeId>& initial_path,
                   LatLon van_pos,
                   LatLon warehouse_pos) {
    van_pos_       = van_pos;
    warehouse_pos_ = warehouse_pos;

    int map_panel_w = static_cast<int>(width_ * 0.75f);

    CoordinateProjector proj;
    proj.setup(graph.minBounds(), graph.maxBounds(),
               width_, height_, map_panel_w);

    const std::vector<NodeId> original_path = initial_path;
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
                    fmt::print("[Renderer] R pressed — triggering replan\n");
                    replan_requested_ = true;
                }
                if (event.key.keysym.sym == SDLK_SPACE) {
                    paused_ = !paused_;
                    fmt::print("[Renderer] {}\n", paused_ ? "Paused" : "Resumed");
                }
                if (event.key.keysym.sym == SDLK_f) {
                    show_fleet_ = !show_fleet_;
                    fmt::print("[Renderer] Fleet markers {}\n", show_fleet_ ? "shown" : "hidden");
                }
                if (event.key.keysym.sym == SDLK_p) {
                    show_path_ = !show_path_;
                    fmt::print("[Renderer] Path {}\n", show_path_ ? "shown" : "hidden");
                }
            }
        }

        if (replan_requested_ && original_path.size() >= 2) {
            replan_requested_ = false;
            NodeId start = original_path.front();
            NodeId goal  = original_path.back();

            std::vector<NodeId> frontier_nodes;
            for (size_t i = 0; i < std::min((size_t)50, original_path.size()); ++i)
                frontier_nodes.push_back(original_path[i]);
            frontier_layer_.triggerReplan(frontier_nodes);

            AStar astar(Heuristic::euclidean());
            auto result = astar.plan(graph, start, goal);
            if (!result.path.empty()) path = result.path;
            replan_count_++;
            fmt::print("[Renderer] Replan #{}: {} frontier nodes, {} path nodes\n",
                       replan_count_, frontier_nodes.size(), result.path.size());
        }

        if (!paused_) {
            SDL_SetRenderDrawColor(renderer_, 18, 18, 18, 255);
            SDL_RenderClear(renderer_);

            map_layer_.draw(renderer_, graph, proj);
            if (show_fleet_)
                scooter_layer_.draw(renderer_, fleet, proj);
            if (show_path_)
                path_layer_.draw(renderer_, path, graph, proj, 0);
            frontier_layer_.draw(renderer_, graph, proj);
            // draw warehouse
            Vec2 wh = proj.project(warehouse_pos_);
            drawWarehouse(renderer_, static_cast<int>(wh.x), static_cast<int>(wh.y));

            // draw van at start position
            Vec2 vp = proj.project(van_pos_);
            drawVan(renderer_, static_cast<int>(vp.x), static_cast<int>(vp.y));

            SDL_RenderPresent(renderer_);
        }
        SDL_Delay(16);
    }
}

void Renderer::handleEvents(MissionController& mission) {}
void Renderer::render(const MissionController& mission,
                      const Graph& graph,
                      const FleetManager& fleet) {}

} // namespace sr