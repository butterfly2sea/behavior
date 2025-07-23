#include <action_libs/rally_plugin.hpp>
#include <set/SetRallyDst.hpp>
#include <behavior_lib/plugin/interface.hpp>
#include <log/Logger.hpp>

namespace zyzn{
namespace plugin{

bool RallyPlugin::parseJsonParam(const Json::Value &params){
  txtLog().info(THISMODULE "Rally plugin parseJsonParam");
  return true;
}

void RallyPlugin::registerNode(BT::BehaviorTreeFactory &factory){
  factory.registerNodeType<set::CSetRallyDst>("SetRallyDst");
  txtLog().info(THISMODULE "Rally plugin registered SetRallyDst node");
}

std::shared_ptr<BasePlugin> create_plugin(){
  return std::make_shared<RallyPlugin>();
}
}
}