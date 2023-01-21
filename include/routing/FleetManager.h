#pragma once
#include "Scooter.h"
#include <vector>
#include <string>
#include <functional>

namespace sr {

class FleetManager {
public:
    // Load fleet from CSV: id,lat,lon,battery_pct,status
    bool loadFromCSV(const std::string& path);

    const std::vector<Scooter>& all()         const;
    std::vector<Scooter>        collectible()  const;
    std::vector<Scooter>        critical()     const;  // battery < 15%

    // Runtime mutations — called by MissionController on operator events
    void markCollected(int id);
    void markNotFound(int id);

    size_t remaining() const;

    // Callback invoked whenever fleet state changes
    using ChangeCallback = std::function<void()>;
    void onFleetChanged(ChangeCallback cb);

private:
    std::vector<Scooter> fleet_;
    ChangeCallback on_change_;
};

} // namespace sr
