#pragma once

#include "types.hpp"
#include <queue>
#include <mutex>
#include <atomic>

namespace behavior_core {

class MessageQueue {
 private:
  std::queue<BehaviorMessage> messages_;
  mutable std::mutex queue_mutex_;
  std::atomic<bool> shutdown_{false};
  size_t max_size_;

 public:
  explicit MessageQueue(size_t max_size = 100) : max_size_(max_size) {}

  ~MessageQueue() {
    shutdown();
  }

  bool push(const BehaviorMessage &msg) {
    std::lock_guard<std::mutex> lock(queue_mutex_);

    if (shutdown_.load()) {
      return false;
    }

    if (messages_.size() >= max_size_) {
      messages_.pop();
    }

    messages_.push(msg);
    return true;
  }

  bool tryPop(BehaviorMessage &msg) {
    std::lock_guard<std::mutex> lock(queue_mutex_);

    if (shutdown_.load() || messages_.empty()) {
      return false;
    }

    msg = messages_.front();
    messages_.pop();
    return true;
  }

  void shutdown() {
    shutdown_.store(true);
  }

  bool isShutdown() const {
    return shutdown_.load();
  }

  size_t size() const {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    return messages_.size();
  }

  bool empty() const {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    return messages_.empty();
  }

  void clear() {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    while (!messages_.empty()) {
      messages_.pop();
    }
  }
};

} // namespace behavior_core
