#include "behavior_node/condition/check_quit_line_loop.hpp"

#include "behavior_node/data/data_cache.hpp"
#include "behavior_node/data/mission_context.hpp"

BT::NodeStatus CheckQuitLineLoop::tick() {
  bool finished = context()->getLoopIndex() >= context()->getLoopCount() * context()->getWaypoints().points.size() - 1;
  return finished ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}