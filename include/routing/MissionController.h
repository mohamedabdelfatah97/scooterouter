#pragma once
#include "FleetManager.h"
#include "RouteOptimizer.h"
#include "../planner/DStarLite.h"
#include "../core/Graph.h"
#include <memory>
#include <functional>

namespace sr {

enum class MissionState { Idle, Routing, Replanning, Complete };

class MissionController {
public:
    MissionController(
        FleetManager& fleet,
        RouteOptimizer& optimizer,
        const Graph& graph);

    void start(const LatLon& operator_start);
    void update();   // called each frame from Renderer

    // Operator events from SDL2 UI clicks
    void markCollected(int scooter_id);
    void markNotFound(int scooter_id);
    void addObstacle(NodeId u, NodeId v);

    // State queries for Renderer layers
    MissionState              state()        const;
    const std::vector<NodeId>& currentPath() const;
    const std::vector<Scooter>& currentRoute() const;
    const Scooter*             nextTarget()  const;
    LatLon                     operatorPos() const;
    int                        replanCount() const;

    // Callbacks for Renderer to react to replan events
    using ReplanCallback = std::function<void(const std::vector<NodeId>&)>;
    void onReplan(ReplanCallback cb);

private:
    void triggerReplan();
    void advanceToNextScooter();

    FleetManager&    fleet_;
    RouteOptimizer&  optimizer_;
    const Graph&     graph_;
    DStarLite        dstar_;

    std::vector<Scooter> current_route_;
    std::vector<NodeId>  current_path_;
    LatLon               operator_pos_  {};
    MissionState         state_         = MissionState::Idle;
    int                  replan_count_  = 0;
    ReplanCallback       on_replan_;
};

} // namespace sr
