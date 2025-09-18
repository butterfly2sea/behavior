#pragma once

#include <behaviortree_cpp/bt_factory.h>
#include <chrono>
#include <memory>
#include <utility>

#include "behavior_node/base/node_dependencies.hpp"
#include <log/Logger.hpp>

/**
 * 通用节点基类模板 - 所有行为树节点的基础类
 */
template<typename Derived, typename BTNodeType>
class NodeBase : public BTNodeType, public DependencyAware {
 public:
  NodeBase(const std::string &name,
           const BT::NodeConfiguration &config,
           NodeDependencies deps)
      : BTNodeType(name, config), DependencyAware(std::move(deps)) {
    txtLog().debug("NodeBase::%s created", name.c_str());
  }

  virtual ~NodeBase() = default;

 protected:
  // 通用的端口值获取方法
  template<typename T>
  BT::Result<T> getInputValue(const std::string& key) const {
    return this->getInput<T>(key);
  }

  template<typename T>
  BT::Result<T> setOutputValue(const std::string& key, const T& value) {
    return this->setOutput(key, value);
  }

  // 日志辅助方法
  void logInfo(const char* format, ...) const {
    va_list args;
    va_start(args, format);
    std::string msg = this->name() + ": " + format;
    txtLog().info(msg.c_str(), args);
    va_end(args);
  }

  void logError(const char* format, ...) const {
    va_list args;
    va_start(args, format);
    std::string msg = this->name() + ": " + format;
    txtLog().error(msg.c_str(), args);
    va_end(args);
  }

  void logDebug(const char* format, ...) const {
    va_list args;
    va_start(args, format);
    std::string msg = this->name() + ": " + format;
    txtLog().debug(msg.c_str(), args);
    va_end(args);
  }
};

/**
 * 同步动作节点基类 - 用于快速执行完成的动作
 */
template<typename Derived>
class SyncActionBase : public NodeBase<Derived, BT::SyncActionNode> {
 public:
  using NodeBase<Derived, BT::SyncActionNode>::NodeBase;

  BT::NodeStatus tick() final override {
    try {
      auto result = static_cast<Derived*>(this)->execute();
      return result;
    } catch (const std::exception& e) {
      this->logError("Exception in tick(): %s", e.what());
      return BT::NodeStatus::FAILURE;
    }
  }

 protected:
  // 派生类需要实现此方法
  virtual BT::NodeStatus execute() = 0;
};

/**
 * 有状态动作节点基类 - 用于需要持续执行的动作
 */
template<typename Derived>
class StatefulActionBase : public NodeBase<Derived, BT::StatefulActionNode> {
 public:
  using NodeBase<Derived, BT::StatefulActionNode>::NodeBase;

 protected:
  std::chrono::steady_clock::time_point start_time_;
  std::chrono::milliseconds timeout_{30000}; // 默认30秒超时

  BT::NodeStatus onStart() override {
    start_time_ = std::chrono::steady_clock::now();
    this->logDebug("Starting stateful action");

    try {
      return static_cast<Derived*>(this)->onActionStart();
    } catch (const std::exception& e) {
      this->logError("Exception in onStart(): %s", e.what());
      return BT::NodeStatus::FAILURE;
    }
  }

  BT::NodeStatus onRunning() override {
    try {
      // 检查超时
      auto elapsed = std::chrono::steady_clock::now() - start_time_;
      if (elapsed > timeout_) {
        this->logError("Action timeout after %ld ms",
                       std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count());
        return BT::NodeStatus::FAILURE;
      }

      return static_cast<Derived*>(this)->onActionRunning();
    } catch (const std::exception& e) {
      this->logError("Exception in onRunning(): %s", e.what());
      return BT::NodeStatus::FAILURE;
    }
  }

  void onHalted() override {
    this->logDebug("Stateful action halted");

    try {
      static_cast<Derived*>(this)->onActionHalted();
    } catch (const std::exception& e) {
      this->logError("Exception in onHalted(): %s", e.what());
    }
  }

  // 派生类需要实现这些方法
  virtual BT::NodeStatus onActionStart() = 0;
  virtual BT::NodeStatus onActionRunning() = 0;
  virtual void onActionHalted() {}

  // 设置超时时间
  void setTimeout(std::chrono::milliseconds timeout) {
    timeout_ = timeout;
  }

  // 检查是否超时
  bool isTimeout() const {
    auto elapsed = std::chrono::steady_clock::now() - start_time_;
    return elapsed > timeout_;
  }

  // 获取运行时间
  std::chrono::milliseconds getElapsedTime() const {
    auto elapsed = std::chrono::steady_clock::now() - start_time_;
    return std::chrono::duration_cast<std::chrono::milliseconds>(elapsed);
  }
};

/**
 * 条件节点基类 - 用于检查各种条件
 */
template<typename Derived>
class ConditionBase : public NodeBase<Derived, BT::ConditionNode> {
 public:
  using NodeBase<Derived, BT::ConditionNode>::NodeBase;

  BT::NodeStatus tick() final override {
    try {
      bool result = static_cast<Derived*>(this)->checkCondition();
      return result ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    } catch (const std::exception& e) {
      this->logError("Exception in tick(): %s", e.what());
      return BT::NodeStatus::FAILURE;
    }
  }

 protected:
  // 派生类需要实现此方法
  virtual bool checkCondition() = 0;
};


// 用于注册节点的宏定义
#define REGISTER_ACTION_NODE(factory, NodeClass, deps) \
  do { \
    BT::NodeBuilder builder = [=](const std::string& name, const BT::NodeConfiguration& config) { \
      return std::make_unique<NodeClass>(name, config, deps); \
    }; \
    factory.registerBuilder<NodeClass>(#NodeClass, builder); \
  } while(0)

#define REGISTER_CONDITION_NODE(factory, NodeClass, deps) \
  do { \
    BT::NodeBuilder builder = [=](const std::string& name, const BT::NodeConfiguration& config) { \
      return std::make_unique<NodeClass>(name, config, deps); \
    }; \
    factory.registerBuilder<NodeClass>(#NodeClass, builder); \
  } while(0)