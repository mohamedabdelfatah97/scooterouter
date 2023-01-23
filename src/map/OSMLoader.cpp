#include "map/OSMLoader.h"
#include "map/CostMap.h"
#include <osmium/io/xml_input.hpp>
#include <osmium/handler.hpp>
#include <osmium/visitor.hpp>
#include <osmium/osm/node.hpp>
#include <osmium/osm/way.hpp>
#include <osmium/index/map/flex_mem.hpp>
#include <osmium/handler/node_locations_for_ways.hpp>
#include <cmath>
#include <unordered_map>
#include <iostream>

namespace sr {

// Haversine distance in km between two lat/lon points
static double haversine(const LatLon& a, const LatLon& b) {
    constexpr double R = 6371.0;
    const double dLat = (b.lat - a.lat) * M_PI / 180.0;
    const double dLon = (b.lon - a.lon) * M_PI / 180.0;
    const double lat1 = a.lat * M_PI / 180.0;
    const double lat2 = b.lat * M_PI / 180.0;
    const double x = std::sin(dLat/2) * std::sin(dLat/2) +
                     std::cos(lat1) * std::cos(lat2) *
                     std::sin(dLon/2) * std::sin(dLon/2);
    return R * 2.0 * std::atan2(std::sqrt(x), std::sqrt(1.0 - x));
}

// OSM handler — processes ways and builds graph edges
using IndexType = osmium::index::map::FlexMem<osmium::unsigned_object_id_type,
                                               osmium::Location>;
using LocationHandler = osmium::handler::NodeLocationsForWays<IndexType>;

struct RoadHandler : public osmium::handler::Handler {
    Graph& graph;
    LatLon& min_bounds;
    LatLon& max_bounds;
    bool bounds_init = false;

    RoadHandler(Graph& g, LatLon& mn, LatLon& mx)
        : graph(g), min_bounds(mn), max_bounds(mx) {}

    void way(const osmium::Way& way) {
        // Only process highway ways
        const char* highway = way.tags()["highway"];
        if (!highway) return;

        std::string hw_tag(highway);

        // Skip pedestrian-only infrastructure
        if (hw_tag == "footway" || hw_tag == "path" ||
            hw_tag == "steps"   || hw_tag == "cycleway") return;

        const double cost_mult = CostMap::multiplierForHighwayTag(hw_tag);
        const bool one_way = (way.tags()["oneway"] &&
                              std::string(way.tags()["oneway"]) == "yes");

        const auto& nodes = way.nodes();
        for (size_t i = 0; i + 1 < nodes.size(); ++i) {
            const auto& n1 = nodes[i];
            const auto& n2 = nodes[i + 1];

            if (!n1.location().valid() || !n2.location().valid()) continue;

            // Add nodes to graph
            auto add_node = [&](const osmium::NodeRef& nr) {
                NodeId nid = static_cast<NodeId>(nr.ref());
                if (!graph.hasNode(nid)) {
                    Node n;
                    n.id      = nid;
                    n.geo.lat = nr.location().lat();
                    n.geo.lon = nr.location().lon();
                    graph.addNode(n);

                    // Track bounds
                    if (!bounds_init) {
                        min_bounds = max_bounds = n.geo;
                        bounds_init = true;
                    } else {
                        min_bounds.lat = std::min(min_bounds.lat, n.geo.lat);
                        min_bounds.lon = std::min(min_bounds.lon, n.geo.lon);
                        max_bounds.lat = std::max(max_bounds.lat, n.geo.lat);
                        max_bounds.lon = std::max(max_bounds.lon, n.geo.lon);
                    }
                }
            };

            add_node(n1);
            add_node(n2);

            NodeId id1 = static_cast<NodeId>(n1.ref());
            NodeId id2 = static_cast<NodeId>(n2.ref());

            LatLon geo1 { n1.location().lat(), n1.location().lon() };
            LatLon geo2 { n2.location().lat(), n2.location().lon() };
            double dist = haversine(geo1, geo2);
            double cost = dist * cost_mult;

            static EdgeId edge_counter = 0;

            // Forward edge
            Edge fwd;
            fwd.id      = edge_counter++;
            fwd.from    = id1;
            fwd.to      = id2;
            fwd.cost    = cost;
            fwd.one_way = one_way;
            graph.addEdge(fwd);

            // Reverse edge if not one-way
            if (!one_way) {
                Edge rev;
                rev.id      = edge_counter++;
                rev.from    = id2;
                rev.to      = id1;
                rev.cost    = cost;
                rev.one_way = false;
                graph.addEdge(rev);
            }
        }
    }
};

bool OSMLoader::load(const std::string& osm_path, Graph& graph) {
    try {
        osmium::io::File input_file{osm_path};
        osmium::io::Reader reader{input_file,
            osmium::osm_entity_bits::node | osmium::osm_entity_bits::way};

        IndexType index;
        LocationHandler location_handler{index};
        RoadHandler road_handler{graph, min_bounds_, max_bounds_};

        osmium::apply(reader, location_handler, road_handler);
        reader.close();

        std::cout << "[OSMLoader] Loaded " << graph.nodeCount()
                  << " nodes, " << graph.edgeCount() << " edges\n";
        return true;

    } catch (const std::exception& e) {
        std::cerr << "[OSMLoader] Error: " << e.what() << "\n";
        return false;
    }
}

LatLon OSMLoader::minBounds() const { return min_bounds_; }
LatLon OSMLoader::maxBounds() const { return max_bounds_; }

} // namespace sr
