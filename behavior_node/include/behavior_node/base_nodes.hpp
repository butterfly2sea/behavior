#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <memory>
#include <utility>

// 前向声明
class ROSCommunicationManager;
class Cache;
class MissionContext;

// 依赖注入容器
struct NodeDependencies {
  std::shared_ptr<ROSCommunicationManager> ros_comm;
  std::shared_ptr<Cache> data_cache;
  std::shared_ptr<MissionContext> mission_context;
};

// 依赖注入接口
class DependencyAware {
 protected:
  NodeDependencies deps_;

 public:
  explicit DependencyAware(NodeDependencies deps) : deps_(std::move(deps)) {}

  // 获取依赖的辅助方法
  auto ros() const { return deps_.ros_comm; }

  auto cache() const { return deps_.data_cache; }

  auto context() const { return deps_.mission_context; }
};

// 通用节点基类模板
template<typename Derived, typename BTNodeType>
class NodeBase : public BTNodeType, public DependencyAware {
 public:
  NodeBase(const std::string &name,
           const BT::NodeConfiguration &config,
           NodeDependencies deps)
      : BTNodeType(name, config), DependencyAware(deps) {}
};

// 同步动作节点基类
template<typename Derived>
class SyncActionBase : public NodeBase<Derived, BT::SyncActionNode> {
 public:
  using NodeBase<Derived, BT::SyncActionNode>::NodeBase;
};

// 状态动作节点基类
template<typename Derived>
class StatefulActionBase : public NodeBase<Derived, BT::StatefulActionNode> {
 public:
  using NodeBase<Derived, BT::StatefulActionNode>::NodeBase;
 protected:
  std::chrono::steady_clock::time_point start_time_;
  std::chrono::milliseconds timeout_{10000};
};

// 条件节点基类
template<typename Derived>
class ConditionBase : public NodeBase<Derived, BT::ConditionNode> {
 public:
  using NodeBase<Derived, BT::ConditionNode>::NodeBase;
};