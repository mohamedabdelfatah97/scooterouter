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
                   const std::vector<NodeId>& path_astar,
                   const std::vector<NodeId>& path_dijkstra,
                   const std::vector<NodeId>& path_bfs,
                   const std::vector<NodeId>& path_dstar,
                   LatLon van_pos,
                   LatLon warehouse_pos) {
    van_pos_       = van_pos;
    warehouse_pos_ = warehouse_pos;

    int map_panel_w = static_cast<int>(width_ * 0.75f);

    CoordinateProjector proj;
    proj.setup(graph.minBounds(), graph.maxBounds(),
               width_, height_, map_panel_w);

    // store all 4 paths
    const std::vector<NodeId>* paths[4] = {
        &path_astar, &path_dijkstra, &path_bfs, &path_dstar
    };
    PathColor colors[4] = {
        PATH_ASTAR, PATH_DIJKSTRA, PATH_BFS, PATH_DSTAR
    };
    const char* algo_names[4] = { "A*", "Dijkstra", "BFS", "D* Lite" };

    // animation state
    size_t anim_index_ = 0;
    int    anim_speed_ = 3;
    bool   anim_done_  = false;

    bool running = true;
    SDL_Event event;

    SDL_Delay(500);
    SDL_FlushEvents(SDL_FIRSTEVENT, SDL_LASTEVENT);

    while (running) {
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) running = false;
            if (event.type == SDL_KEYDOWN) {
                if (event.key.keysym.sym == SDLK_ESCAPE) running = false;
                if (event.key.keysym.sym == SDLK_SPACE) {
                    paused_ = !paused_;
                    fmt::print("[Renderer] {}\n", paused_ ? "Paused" : "Resumed");
                }
                if (event.key.keysym.sym == SDLK_f) {
                    show_fleet_ = !show_fleet_;
                    fmt::print("[Renderer] Fleet {}\n", show_fleet_ ? "shown" : "hidden");
                }
                if (event.key.keysym.sym == SDLK_p) {
                    show_path_ = !show_path_;
                    fmt::print("[Renderer] Path {}\n", show_path_ ? "shown" : "hidden");
                }
                // algorithm switching
                if (event.key.keysym.sym == SDLK_1) {
                    active_algo_ = 1; anim_index_ = 0; anim_done_ = false;
                    fmt::print("[Renderer] Algorithm: A* (blue)\n");
                }
                if (event.key.keysym.sym == SDLK_2) {
                    active_algo_ = 2; anim_index_ = 0; anim_done_ = false;
                    fmt::print("[Renderer] Algorithm: Dijkstra (green)\n");
                }
                if (event.key.keysym.sym == SDLK_3) {
                    active_algo_ = 3; anim_index_ = 0; anim_done_ = false;
                    fmt::print("[Renderer] Algorithm: BFS (pink)\n");
                }
                if (event.key.keysym.sym == SDLK_4) {
                    active_algo_ = 4; anim_index_ = 0; anim_done_ = false;
                    fmt::print("[Renderer] Algorithm: D* Lite (yellow)\n");
                }
                if (event.key.keysym.sym == SDLK_0) {
                    active_algo_ = 0; anim_index_ = 0; anim_done_ = true;
                    fmt::print("[Renderer] Algorithm: All overlaid\n");
                }
                // speed controls
                if (event.key.keysym.sym == SDLK_EQUALS ||
                    event.key.keysym.sym == SDLK_PLUS) {
                    anim_speed_ = std::min(anim_speed_ + 1, 20);
                    fmt::print("[Renderer] Speed: {}\n", anim_speed_);
                }
                if (event.key.keysym.sym == SDLK_MINUS) {
                    anim_speed_ = std::max(anim_speed_ - 1, 1);
                    fmt::print("[Renderer] Speed: {}\n", anim_speed_);
                }
                // restart
                if (event.key.keysym.sym == SDLK_r) {
                    anim_index_ = 0; anim_done_ = false;
                    fmt::print("[Renderer] Restarted\n");
                }
            }
        }

        if (!paused_) {
            // advance animation for single algorithm mode
            if (active_algo_ > 0 && !anim_done_) {
                const auto& cur_path = *paths[active_algo_ - 1];
                anim_index_ += anim_speed_;
                if (anim_index_ >= cur_path.size()) {
                    anim_index_ = cur_path.size();
                    if (!anim_done_) {
                        anim_done_ = true;
                        fmt::print("[Renderer] {} route complete!\n",
                                   algo_names[active_algo_ - 1]);
                    }
                }
            }

            SDL_SetRenderDrawColor(renderer_, 18, 18, 18, 255);
            SDL_RenderClear(renderer_);

            map_layer_.draw(renderer_, graph, proj);

            if (show_fleet_)
                scooter_layer_.draw(renderer_, fleet, proj);

            if (show_path_) {
                if (active_algo_ == 0) {
                    PathOffset offsets[4] = {
                        OFFSET_ASTAR, OFFSET_DIJKSTRA, OFFSET_BFS, OFFSET_DSTAR
                    };
                    for (int i = 0; i < 4; ++i)
                        path_layer_.draw(renderer_, *paths[i], graph, proj, 0,
                                         colors[i], offsets[i]);
                } else {
                    // draw animated single algorithm
                    const auto& cur_path = *paths[active_algo_ - 1];
                    if (anim_index_ > 0) {
                        std::vector<NodeId> visible(
                            cur_path.begin(),
                            cur_path.begin() + anim_index_);
                        path_layer_.draw(renderer_, visible, graph, proj, 0,
                                         colors[active_algo_ - 1]);
                    }
                }
            }

            // draw warehouse
            Vec2 wh = proj.project(warehouse_pos_);
            if (warehouse_texture_) {
                SDL_Rect dst { static_cast<int>(wh.x) - 20,
                               static_cast<int>(wh.y) - 20, 40, 40 };
                SDL_RenderCopy(renderer_, warehouse_texture_, nullptr, &dst);
            }

            // draw van at current animated position
            if (van_texture_ && active_algo_ > 0) {
                const auto& cur_path = *paths[active_algo_ - 1];
                if (!cur_path.empty()) {
                    NodeId cur_node = cur_path[std::min(anim_index_, cur_path.size()-1)];
                    if (graph.hasNode(cur_node)) {
                        Vec2 vp = proj.project(graph.getNode(cur_node).geo);
                        SDL_Rect dst { static_cast<int>(vp.x) - 20,
                                       static_cast<int>(vp.y) - 12, 40, 24 };
                        SDL_RenderCopy(renderer_, van_texture_, nullptr, &dst);
                    }
                }
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