#pragma once
#include "../core/Types.h"
#include <string>

namespace sr {

enum class ScooterStatus { Available, Damaged, Missing, Collected };

struct Scooter {
    int           id          = 0;
    LatLon        geo         {};
    int           battery_pct = 0;
    ScooterStatus status      = ScooterStatus::Available;
    NodeId        nearest_node = INVALID_NODE; // snapped to road graph

    bool isCritical()  const { return battery_pct < 15; }
    bool isCollectible() const {
        return status == ScooterStatus::Available ||
               status == ScooterStatus::Damaged;
    }
};

} // namespace sr
