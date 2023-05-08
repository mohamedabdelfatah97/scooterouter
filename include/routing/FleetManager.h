#pragma once
#include "Scooter.h"
#include "../core/Graph.h"
#include <vector>
#include <string>
#include <functional>
#include <random>

namespace sr {

class FleetManager {
public:
    // Load fleet from CSV: id,lat,lon,battery_pct,status
    bool loadFromCSV(const std::string& path);

    // Generate N scooters at random graph nodes
    void generateRandom(const Graph& graph, int count, unsigned int seed = 42);

    const std::vector<Scooter>& all()         const;
    std::vector<Scooter>        collectible()  const;
    std::vector<Scooter>        critical()     const;

    void markCollected(int id);
    void markNotFound(int id);

    size_t remaining() const;

    using ChangeCallback = std::function<void()>;
    void onFleetChanged(ChangeCallback cb);

private:
    std::vector<Scooter> fleet_;
    ChangeCallback on_change_;
};

} // namespace sr