// ================================ message_queue.hpp ================================
#pragma once

#include <queue>
#include <mutex>
#include <condition_variable>
#include <memory>
#include <atomic>
#include <chrono>
#include <log/Logger.hpp>

namespace behavior_core {

// 前向声明
struct BaseMessage;
struct TreeMessage;
struct ControlMessage;
struct AttackMessage;

class MessageQueue {
 private:
  std::queue<std::shared_ptr<BaseMessage>> queue_;
  mutable std::mutex mutex_;
  std::condition_variable condition_;

  std::atomic<bool> shutdown_requested_{false};
  std::atomic<size_t> max_size_;
  std::atomic<size_t> current_size_{0};

  // 统计信息
  std::atomic<size_t> total_pushed_{0};
  std::atomic<size_t> total_popped_{0};
  std::atomic<size_t> total_dropped_{0};

 public:
  explicit MessageQueue(size_t max_size = 100) : max_size_(max_size) {
    txtLog().info(THISMODULE "MessageQueue created with max_size: %zu", max_size);
  }

  ~MessageQueue() {
    shutdown();
  }

  // ================================ 基本操作 ================================

  bool push(std::shared_ptr<BaseMessage> message) {
    if (shutdown_requested_.load()) {
      return false;
    }

    std::lock_guard<std::mutex> lock(mutex_);

    // 检查队列是否已满
    if (queue_.size() >= max_size_.load()) {
      // 移除最老的消息为新消息腾出空间
      queue_.pop();
      total_dropped_.fetch_add(1);
      txtLog().warnning(THISMODULE "MessageQueue full, dropped oldest message");
    }

    queue_.push(message);
    current_size_.store(queue_.size());
    total_pushed_.fetch_add(1);

    condition_.notify_one();
    return true;
  }

  std::shared_ptr<BaseMessage> pop(std::chrono::milliseconds timeout = std::chrono::milliseconds(100)) {
    std::unique_lock<std::mutex> lock(mutex_);

    if (condition_.wait_for(lock, timeout, [this] {
      return !queue_.empty() || shutdown_requested_.load();
    })) {
      if (!queue_.empty()) {
        auto message = queue_.front();
        queue_.pop();
        current_size_.store(queue_.size());
        total_popped_.fetch_add(1);
        return message;
      }
    }

    return nullptr;
  }

  std::shared_ptr<BaseMessage> tryPop() {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!queue_.empty()) {
      auto message = queue_.front();
      queue_.pop();
      current_size_.store(queue_.size());
      total_popped_.fetch_add(1);
      return message;
    }

    return nullptr;
  }

  // ================================ 状态查询 ================================

  bool empty() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_.empty();
  }

  size_t size() const {
    return current_size_.load();
  }

  size_t maxSize() const {
    return max_size_.load();
  }

  void setMaxSize(size_t new_max_size) {
    std::lock_guard<std::mutex> lock(mutex_);
    max_size_.store(new_max_size);

    // 如果当前队列大小超过新的最大值，移除多余的消息
    while (queue_.size() > new_max_size) {
      queue_.pop();
      total_dropped_.fetch_add(1);
    }
    current_size_.store(queue_.size());
  }

  bool isHealthy() const {
    // 检查队列是否健康运行
    return !shutdown_requested_.load() && current_size_.load() < max_size_.load();
  }

  // ================================ 统计信息 ================================

  struct Statistics {
    size_t total_pushed;
    size_t total_popped;
    size_t total_dropped;
    size_t current_size;
    size_t max_size;
    double drop_rate;
  };

  Statistics getStatistics() const {
    Statistics stats;
    stats.total_pushed = total_pushed_.load();
    stats.total_popped = total_popped_.load();
    stats.total_dropped = total_dropped_.load();
    stats.current_size = current_size_.load();
    stats.max_size = max_size_.load();
    stats.drop_rate = stats.total_pushed > 0 ?
                      static_cast<double>(stats.total_dropped) / stats.total_pushed : 0.0;
    return stats;
  }

  void printStatistics() const {
    auto stats = getStatistics();
    txtLog().info(THISMODULE "MessageQueue Stats - Pushed: %zu, Popped: %zu, Dropped: %zu, "
                             "Current: %zu/%zu, Drop Rate: %.2f%%",
                  stats.total_pushed, stats.total_popped, stats.total_dropped,
                  stats.current_size, stats.max_size, stats.drop_rate * 100.0);
  }

  // ================================ 关闭操作 ================================

  void shutdown() {
    if (shutdown_requested_.exchange(true)) {
      return; // 已经关闭
    }

    txtLog().info(THISMODULE "Shutting down MessageQueue");

    {
      std::lock_guard<std::mutex> lock(mutex_);
      // 清空队列
      while (!queue_.empty()) {
        queue_.pop();
      }
      current_size_.store(0);
    }

    condition_.notify_all();
    txtLog().info(THISMODULE "MessageQueue shutdown complete");
  }

  // ================================ 类型安全的便利方法 ================================

  template<typename MessageT>
  bool pushTyped(std::shared_ptr<MessageT> message) {
    static_assert(std::is_base_of_v<BaseMessage, MessageT>,
                  "MessageT must derive from BaseMessage");
    return push(std::static_pointer_cast<BaseMessage>(message));
  }

  template<typename MessageT>
  std::shared_ptr<MessageT> popTyped(std::chrono::milliseconds timeout = std::chrono::milliseconds(100)) {
    static_assert(std::is_base_of_v<BaseMessage, MessageT>,
                  "MessageT must derive from BaseMessage");
    auto message = pop(timeout);
    return std::dynamic_pointer_cast<MessageT>(message);
  }

  template<typename MessageT>
  std::shared_ptr<MessageT> tryPopTyped() {
    static_assert(std::is_base_of_v<BaseMessage, MessageT>,
                  "MessageT must derive from BaseMessage");
    auto message = tryPop();
    return std::dynamic_pointer_cast<MessageT>(message);
  }

  // ================================ 消息过滤 ================================

  std::vector<std::shared_ptr<BaseMessage>> popAllOfType(MessageType type) {
    std::vector<std::shared_ptr<BaseMessage>> messages;
    std::lock_guard<std::mutex> lock(mutex_);

    std::queue<std::shared_ptr<BaseMessage>> temp_queue;

    while (!queue_.empty()) {
      auto message = queue_.front();
      queue_.pop();

      if (message->type == type) {
        messages.push_back(message);
        total_popped_.fetch_add(1);
      } else {
        temp_queue.push(message);
      }
    }

    // 将不匹配的消息放回队列
    queue_ = temp_queue;
    current_size_.store(queue_.size());

    return messages;
  }

  size_t countMessagesOfType(MessageType type) const {
    std::lock_guard<std::mutex> lock(mutex_);

    size_t count = 0;
    std::queue<std::shared_ptr<BaseMessage>> temp_queue = queue_;

    while (!temp_queue.empty()) {
      if (temp_queue.front()->type == type) {
        count++;
      }
      temp_queue.pop();
    }

    return count;
  }

  void clearMessagesOfType(MessageType type) {
    std::lock_guard<std::mutex> lock(mutex_);

    std::queue<std::shared_ptr<BaseMessage>> temp_queue;

    while (!queue_.empty()) {
      auto message = queue_.front();
      queue_.pop();

      if (message->type != type) {
        temp_queue.push(message);
      } else {
        total_dropped_.fetch_add(1);
      }
    }

    queue_ = temp_queue;
    current_size_.store(queue_.size());
  }
};

} // namespace behavior_core