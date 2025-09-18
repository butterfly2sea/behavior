#pragma once

#include <memory>

// 前向声明
class ROSCommunicationManager;
class Cache;
class MissionContext;

namespace behavior_core {
class MessageQueue;
}

/**
 * 依赖注入容器 - 包含所有行为节点需要的共享资源
 */
struct NodeDependencies {
  std::shared_ptr<ROSCommunicationManager> ros_comm;
  std::shared_ptr<Cache> data_cache;
  std::shared_ptr<MissionContext> mission_context;
  std::shared_ptr<behavior_core::MessageQueue> message_queue;

  NodeDependencies() = default;

  NodeDependencies(std::shared_ptr<ROSCommunicationManager> ros,
                   std::shared_ptr<Cache> cache,
                   std::shared_ptr<MissionContext> context,
                   std::shared_ptr<behavior_core::MessageQueue> queue)
      : ros_comm(std::move(ros)),
        data_cache(std::move(cache)),
        mission_context(std::move(context)),
        message_queue(std::move(queue)) {}

  // 验证所有依赖是否有效
  bool isValid() const {
    return ros_comm && data_cache && mission_context && message_queue;
  }
};

/**
 * 依赖注入接口 - 为行为节点提供统一的依赖访问方式
 */
class DependencyAware {
 protected:
  NodeDependencies deps_;

 public:
  explicit DependencyAware(NodeDependencies deps) : deps_(std::move(deps)) {
    if (!deps_.isValid()) {
      throw std::invalid_argument("Invalid node dependencies provided");
    }
  }

  virtual ~DependencyAware() = default;

  // 获取依赖的辅助方法
  auto ros() const { return deps_.ros_comm; }
  auto cache() const { return deps_.data_cache; }
  auto context() const { return deps_.mission_context; }
  auto messageQueue() const { return deps_.message_queue; }

  // 检查依赖是否有效
  bool hasDependencies() const { return deps_.isValid(); }
};