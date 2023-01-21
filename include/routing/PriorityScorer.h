#pragma once
#include "Scooter.h"
#include "../core/Types.h"
#include <vector>

namespace sr {

class PriorityScorer {
public:
    struct Weights {
        double battery  = 0.6;
        double status   = 0.3;
        double distance = 0.1;
    };

    PriorityScorer();
    explicit PriorityScorer(Weights w);

    double score(const Scooter& s, double distance_km) const;

    std::vector<Scooter> ranked(
        const std::vector<Scooter>& fleet,
        const LatLon& operator_pos) const;

private:
    Weights weights_;
};

} // namespace sr
