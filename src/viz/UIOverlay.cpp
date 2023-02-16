#include "viz/UIOverlay.h"

namespace sr {

UIOverlay::~UIOverlay() {
    if (font_)       TTF_CloseFont(font_);
    if (font_small_) TTF_CloseFont(font_small_);
}

bool UIOverlay::init(SDL_Renderer* r) { return true; }

void UIOverlay::draw(SDL_Renderer* r,
                     const MissionController& mission,
                     const FleetManager& fleet,
                     int panel_x, int panel_w, int screen_h) {}

} // namespace sr