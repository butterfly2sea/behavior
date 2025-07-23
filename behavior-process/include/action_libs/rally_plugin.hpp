#ifndef ZYZN_PLUGIN_RALLY_PLUGIN_HPP
#define ZYZN_PLUGIN_RALLY_PLUGIN_HPP
#include <behavior_lib/plugin/base_plugin.hpp>

namespace zyzn{
namespace plugin{
/**
 * @brief Rally集结任务插件
 * @details 实现Rally集结任务，使多架无人机飞到指定集结点并保持编队偏移
 */
class RallyPlugin: public BasePlugin{
 public:

  /**
   * @brief 解析Rally任务的JSON参数
   * @param params JSON参数数组
   * @return 是否解析成功
   */
  bool parseJsonParam(const Json::Value &params);

  /**
   * @brief 注册Rally相关的行为树节点
   * @param factory 行为树工厂
   */
  void registerNode(BT::BehaviorTreeFactory &factory);

};

}
}

#endif