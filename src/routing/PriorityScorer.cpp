#include "routing/PriorityScorer.h"
#include <cmath>
#include <algorithm>

namespace sr {

PriorityScorer::PriorityScorer() : weights_({}) {}

PriorityScorer::PriorityScorer(Weights w) : weights_(w) {}

double PriorityScorer::score(const Scooter& s, double distance_km) const {
    double battery_score  = weights_.battery  * (100.0 - s.battery_pct);
    double status_score   = weights_.status   * (s.status == ScooterStatus::Damaged ? 100.0 : 0.0);
    double distance_score = weights_.distance * (1.0 / (distance_km + 0.1));
    return battery_score + status_score + distance_score;
}

std::vector<Scooter> PriorityScorer::ranked(
    const std::vector<Scooter>& fleet,
    const LatLon& operator_pos) const {

    std::vector<std::pair<double, Scooter>> scored;
    for (const auto& s : fleet) {
        double dlat = s.geo.lat - operator_pos.lat;
        double dlon = s.geo.lon - operator_pos.lon;
        double dist = std::sqrt(dlat*dlat + dlon*dlon) * 111.0;
        scored.push_back({ score(s, dist), s });
    }
    std::sort(scored.begin(), scored.end(),
        [](const auto& a, const auto& b) { return a.first > b.first; });

    std::vector<Scooter> result;
    for (const auto& [sc, s] : scored) result.push_back(s);
    return result;
}

} // namespace sr