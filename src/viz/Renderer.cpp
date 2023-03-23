#include "viz/Renderer.h"
#include "map/CoordinateProjector.h"
#include "planner/AStar.h"
#include "planner/Heuristic.h"
#include <fmt/core.h>
#include <SDL2/SDL_image.h>

namespace sr {

Renderer::Renderer(int width, int height)
    : width_(width), height_(height) {}

Renderer::~Renderer() {
    if (van_texture_)       SDL_DestroyTexture(van_texture_);
    if (warehouse_texture_) SDL_DestroyTexture(warehouse_texture_);
    IMG_Quit();
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

    IMG_Init(IMG_INIT_PNG);

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

    SDL_RaiseWindow(window_);

    renderer_ = SDL_CreateRenderer(
        window_, -1,
        SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC
    );
    if (!renderer_) {
        fmt::print("[Renderer] CreateRenderer failed: {}\n", SDL_GetError());
        return false;
    }

    SDL_SetRenderDrawBlendMode(renderer_, SDL_BLENDMODE_BLEND);

    // load PNG icons
    SDL_Surface* van_surf = IMG_Load("assets/van.png");
    SDL_Surface* wh_surf  = IMG_Load("assets/warehouse.png");
    if (van_surf) {
        van_texture_ = SDL_CreateTextureFromSurface(renderer_, van_surf);
        SDL_FreeSurface(van_surf);
        fmt::print("[Renderer] van icon loaded\n");
    } else {
        fmt::print("[Renderer] van icon not found: {}\n", IMG_GetError());
    }
    if (wh_surf) {
        warehouse_texture_ = SDL_CreateTextureFromSurface(renderer_, wh_surf);
        SDL_FreeSurface(wh_surf);
        fmt::print("[Renderer] warehouse icon loaded\n");
    } else {
        fmt::print("[Renderer] warehouse icon not found: {}\n", IMG_GetError());
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

            // draw warehouse icon
            Vec2 wh = proj.project(warehouse_pos_);
            if (warehouse_texture_) {
                SDL_Rect dst { static_cast<int>(wh.x) - 20,
                               static_cast<int>(wh.y) - 20, 40, 40 };
                SDL_RenderCopy(renderer_, warehouse_texture_, nullptr, &dst);
            }

            // draw van icon
            Vec2 vp = proj.project(van_pos_);
            if (van_texture_) {
                SDL_Rect dst { static_cast<int>(vp.x) - 20,
                               static_cast<int>(vp.y) - 12, 40, 24 };
                SDL_RenderCopy(renderer_, van_texture_, nullptr, &dst);
            }

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