#pragma once

#include <queue>
#include <mutex>
#include <condition_variable>
#include <optional>
#include <chrono>

#include "behavior_node/core/types.hpp"

namespace behavior_core {

/**
 * 线程安全的行为消息队列
 */
class MessageQueue {
 private:
  std::queue<BehaviorMessage> queue_;
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  bool shutdown_ = false;
  size_t max_size_;

 public:
  explicit MessageQueue(size_t max_size = 100) : max_size_(max_size) {}

  ~MessageQueue() {
    shutdown();
  }

  /**
   * 向队列添加消息
   * @param message 要添加的消息
   * @return 是否成功添加
   */
  bool push(const BehaviorMessage& message) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (shutdown_) {
      return false;
    }

    // 如果队列已满，移除最旧的消息
    if (queue_.size() >= max_size_) {
      queue_.pop();
    }

    queue_.push(message);
    condition_.notify_one();
    return true;
  }

  /**
   * 从队列获取消息（阻塞版本）
   * @return 消息（如果队列已关闭则返回空）
   */
  std::optional<BehaviorMessage> pop() {
    std::unique_lock<std::mutex> lock(mutex_);

    condition_.wait(lock, [this] { return !queue_.empty() || shutdown_; });

    if (shutdown_ && queue_.empty()) {
      return std::nullopt;
    }

    auto message = queue_.front();
    queue_.pop();
    return message;
  }

  /**
   * 从队列获取消息（非阻塞版本）
   * @return 消息（如果队列为空则返回空）
   */
  std::optional<BehaviorMessage> tryPop() {
    std::lock_guard<std::mutex> lock(mutex_);

    if (queue_.empty() || shutdown_) {
      return std::nullopt;
    }

    auto message = queue_.front();
    queue_.pop();
    return message;
  }

  /**
   * 超时等待消息
   * @param timeout 超时时间
   * @return 消息（如果超时或队列已关闭则返回空）
   */
  std::optional<BehaviorMessage> waitFor(
      const std::chrono::milliseconds& timeout) {
    std::unique_lock<std::mutex> lock(mutex_);

    if (condition_.wait_for(lock, timeout,
                            [this] { return !queue_.empty() || shutdown_; })) {
      if (shutdown_ && queue_.empty()) {
        return std::nullopt;
      }

      auto message = queue_.front();
      queue_.pop();
      return message;
    }

    return std::nullopt;
  }

  /**
   * 获取队列大小
   * @return 当前队列中的消息数量
   */
  size_t size() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_.size();
  }

  /**
   * 检查队列是否为空
   * @return 队列是否为空
   */
  bool empty() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_.empty();
  }

  /**
   * 清空队列
   */
  void clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    std::queue<BehaviorMessage> empty;
    queue_.swap(empty);
  }

  /**
   * 关闭队列
   */
  void shutdown() {
    std::lock_guard<std::mutex> lock(mutex_);
    shutdown_ = true;
    condition_.notify_all();
  }

  /**
   * 检查队列是否已关闭
   * @return 队列是否已关闭
   */
  bool isShutdown() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return shutdown_;
  }
};

} // namespace behavior_core