#include "viz/Renderer.h"
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
        SDL_WINDOW_SHOWN
    );
    if (!window_) {
        fmt::print("[Renderer] CreateWindow failed: {}\n", SDL_GetError());
        return false;
    }

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
    bool running = true;
    SDL_Event event;

    while (running) {
        // handle events
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) running = false;
            if (event.type == SDL_KEYDOWN &&
                event.key.keysym.sym == SDLK_ESCAPE) running = false;
        }

        // clear
        SDL_SetRenderDrawColor(renderer_, 18, 18, 18, 255);
        SDL_RenderClear(renderer_);

        // present
        SDL_RenderPresent(renderer_);
        SDL_Delay(16); // ~60fps
    }
}

void Renderer::handleEvents(MissionController& mission) {}
void Renderer::render(const MissionController& mission,
                      const Graph& graph,
                      const FleetManager& fleet) {}

} // namespace sr