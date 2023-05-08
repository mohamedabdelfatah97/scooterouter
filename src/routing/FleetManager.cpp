#include "routing/FleetManager.h"
#include <fstream>
#include <sstream>
#include <iostream>

namespace sr {

bool FleetManager::loadFromCSV(const std::string& path) {
    std::ifstream file(path);
    if (!file.is_open()) {
        std::cerr << "[FleetManager] Cannot open: " << path << "\n";
        return false;
    }

    std::string line;
    std::getline(file, line); // skip header

    while (std::getline(file, line)) {
        if (line.empty()) continue;
        std::stringstream ss(line);
        std::string token;

        Scooter s;
        std::getline(ss, token, ','); s.id          = std::stoi(token);
        std::getline(ss, token, ','); s.geo.lat      = std::stod(token);
        std::getline(ss, token, ','); s.geo.lon      = std::stod(token);
        std::getline(ss, token, ','); s.battery_pct  = std::stoi(token);
        std::getline(ss, token, ',');
        if (token == "damaged") s.status = ScooterStatus::Damaged;
        else                    s.status = ScooterStatus::Available;

        fleet_.push_back(s);
    }
    return !fleet_.empty();
}

const std::vector<Scooter>& FleetManager::all() const { return fleet_; }

std::vector<Scooter> FleetManager::collectible() const {
    std::vector<Scooter> result;
    for (const auto& s : fleet_)
        if (s.isCollectible()) result.push_back(s);
    return result;
}

std::vector<Scooter> FleetManager::critical() const {
    std::vector<Scooter> result;
    for (const auto& s : fleet_)
        if (s.isCritical() && s.isCollectible()) result.push_back(s);
    return result;
}

void FleetManager::markCollected(int id) {
    for (auto& s : fleet_)
        if (s.id == id) { s.status = ScooterStatus::Collected; break; }
    if (on_change_) on_change_();
}

void FleetManager::markNotFound(int id) {
    for (auto& s : fleet_)
        if (s.id == id) { s.status = ScooterStatus::Missing; break; }
    if (on_change_) on_change_();
}

size_t FleetManager::remaining() const {
    return collectible().size();
}

void FleetManager::onFleetChanged(ChangeCallback cb) {
    on_change_ = std::move(cb);
}

void FleetManager::generateRandom(const Graph& graph, int count, unsigned int seed) {
    std::mt19937 rng(seed);

    // collect all node ids
    std::vector<NodeId> node_ids;
    node_ids.reserve(graph.allNodes().size());
    for (const auto& [id, node] : graph.allNodes())
        node_ids.push_back(id);

    std::uniform_int_distribution<size_t> node_dist(0, node_ids.size() - 1);
    std::uniform_int_distribution<int>    batt_dist(3, 95);
    std::uniform_int_distribution<int>    status_dist(0, 4); // 0-3 available, 4 damaged

    fleet_.clear();
    for (int i = 0; i < count; ++i) {
        Scooter s;
        s.id          = i + 1;
        s.geo         = graph.getNode(node_ids[node_dist(rng)]).geo;
        s.battery_pct = batt_dist(rng);
        s.status      = (status_dist(rng) == 4) ? ScooterStatus::Damaged
                                                 : ScooterStatus::Available;
        fleet_.push_back(s);
    }
}
} // namespace sr
