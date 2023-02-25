#include "routing/MissionController.h"

namespace sr {

MissionController::MissionController(
    FleetManager& fleet,
    RouteOptimizer& optimizer,
    const Graph& graph)
    : fleet_(fleet), optimizer_(optimizer), graph_(graph) {}

void MissionController::start(const LatLon& operator_start) {
    operator_pos_ = operator_start;
    state_ = MissionState::Routing;
}

void MissionController::update() {}
void MissionController::markCollected(int id) { fleet_.markCollected(id); }
void MissionController::markNotFound(int id)  { fleet_.markNotFound(id); }
void MissionController::addObstacle(NodeId u, NodeId v) {}
void MissionController::triggerReplan() { replan_count_++; }
void MissionController::advanceToNextScooter() {}

MissionState              MissionController::state()        const { return state_; }
const std::vector<NodeId>& MissionController::currentPath() const { return current_path_; }
const std::vector<Scooter>& MissionController::currentRoute() const { return current_route_; }
const Scooter*             MissionController::nextTarget()   const { return nullptr; }
LatLon                     MissionController::operatorPos()  const { return operator_pos_; }
int                        MissionController::replanCount()  const { return replan_count_; }

void MissionController::onReplan(ReplanCallback cb) { on_replan_ = std::move(cb); }

} // namespace sr