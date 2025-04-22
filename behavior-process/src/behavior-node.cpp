#include <cstdio>
#include <string>
#include <rclcpp/rclcpp.hpp>

#include <behavior_lib/control/TaskDecision.hpp>
#include <behavior_lib/manage/ActionsMgr.hpp>
#include <behavior_lib/manage/action_plugin_mgr.hpp>
#include <behavior_lib/info/Param.hpp>


int main(int argc, char ** argv)
{
  std::string exePath=argv[0];
  exePath.resize(exePath.find_last_of("/")+1);

  rclcpp::init(argc,argv);
  std::shared_ptr<zyzn::manage::CActionsMgr> actMgr = std::make_shared<zyzn::manage::CActionsMgr>();
  std::shared_ptr<zyzn::manage::ActionPluginMgr>plgMgr = std::make_shared<zyzn::manage::ActionPluginMgr>();
  
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"start behavior-node build at:%s %s",__DATE__,__TIME__);
  std::shared_ptr<rclcpp::Node> treeNode = std::make_shared<rclcpp::Node>("tree_node");
  std::shared_ptr<rclcpp::Node> routeNode = std::make_shared<rclcpp::Node>("route_node");
  auto decisionNode = std::make_shared<zyzn::ctrl::CTaskDecision>("decision_node",actMgr,plgMgr);
  zyzn::info::CParam::rosNode() = treeNode;
  decisionNode.get()->setNode(routeNode);
  decisionNode.get()->regisAllNode(exePath);
  rclcpp::executors::MultiThreadedExecutor exetor;
  exetor.add_node(decisionNode);
  exetor.add_node(treeNode);
  //exetor.add_node(routeNode);
  exetor.spin();
  //rclcpp::spin(decisionNode);
  rclcpp::shutdown();
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"exit behavior-node\n");
  
  return 0;
}
