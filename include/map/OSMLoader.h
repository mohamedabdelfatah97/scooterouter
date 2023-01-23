#pragma once
#include "../core/Graph.h"
#include <string>

namespace sr {

class OSMLoader {
public:
    // Parse .osm XML and populate graph.
    // Extracts highway nodes + ways, respects oneway tags.
    // Returns false if file not found or parse error.
    bool load(const std::string& osm_path, Graph& graph);

    // Bounding box of loaded map (for CoordinateProjector)
    LatLon minBounds() const;
    LatLon maxBounds() const;

private:
    LatLon min_bounds_ {};
    LatLon max_bounds_ {};
};

} // namespace sr
