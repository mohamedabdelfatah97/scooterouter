#pragma once
#include "../routing/MissionController.h"
#include "../routing/FleetManager.h"
#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>

namespace sr {

class UIOverlay {
public:
    bool init(SDL_Renderer* r);
    void draw(SDL_Renderer* r,
              const MissionController& mission,
              const FleetManager& fleet,
              int panel_x, int panel_w, int screen_h);
    ~UIOverlay();

private:
    TTF_Font* font_       = nullptr;
    TTF_Font* font_small_ = nullptr;
};

} // namespace sr
