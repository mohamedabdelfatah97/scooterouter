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

    fmt::print("[Renderer] Window {}x{} initialized\n", width_, height_);
    return true;
}

void Renderer::run(MissionController& mission, const Graph& graph,
                   const FleetManager& fleet) {
    int map_panel_w = static_cast<int>(width_ * 0.75f);

    CoordinateProjector proj;
    proj.setup(graph.minBounds(), graph.maxBounds(),
               width_, height_, map_panel_w);

    bool running = true;
    SDL_Event event;

    SDL_Delay(500);
    fmt::print("[Renderer] entering event loop\n");

    while (running) {
        while (SDL_PollEvent(&event)) {
            fmt::print("[Renderer] event type: {}\n", event.type);
            if (event.type == SDL_QUIT) {
                fmt::print("[Renderer] SDL_QUIT received\n");
                running = false;
            }
            if (event.type == SDL_KEYDOWN &&
                event.key.keysym.sym == SDLK_ESCAPE) running = false;
        }

        SDL_SetRenderDrawColor(renderer_, 18, 18, 18, 255);
        SDL_RenderClear(renderer_);
        map_layer_.draw(renderer_, graph, proj);
        SDL_RenderPresent(renderer_);
        SDL_Delay(16);
    }
    fmt::print("[Renderer] event loop exited\n");
}

void Renderer::handleEvents(MissionController& mission) {}
void Renderer::render(const MissionController& mission,
                      const Graph& graph,
                      const FleetManager& fleet) {}

} // namespace sr