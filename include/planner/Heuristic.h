#pragma once
#include "../core/Types.h"
#include "../core/Node.h"
#include <cmath>
#include <functional>
#include <string>

namespace sr {

using HeuristicFn = std::function<Cost(const Node&, const Node&)>;

struct Heuristic {
    static HeuristicFn euclidean() {
        return [](const Node& a, const Node& b) -> Cost {
            double dlat = a.geo.lat - b.geo.lat;
            double dlon = a.geo.lon - b.geo.lon;
            return std::sqrt(dlat*dlat + dlon*dlon) * 111.0;
        };
    }

    static HeuristicFn manhattan() {
        return [](const Node& a, const Node& b) -> Cost {
            return (std::abs(a.geo.lat - b.geo.lat) +
                    std::abs(a.geo.lon - b.geo.lon)) * 111.0;
        };
    }

    static HeuristicFn octile() {
        return [](const Node& a, const Node& b) -> Cost {
            double dlat = std::abs(a.geo.lat - b.geo.lat);
            double dlon = std::abs(a.geo.lon - b.geo.lon);
            return 111.0 * (dlat + dlon - 0.586 * std::min(dlat, dlon));
        };
    }

    static HeuristicFn zero() {
        return [](const Node&, const Node&) -> Cost { return 0.0; };
    }

    static HeuristicFn fromString(const std::string& name) {
        if (name == "manhattan") return manhattan();
        if (name == "octile")    return octile();
        if (name == "zero")      return zero();
        return euclidean();
    }
};

} // namespace sr