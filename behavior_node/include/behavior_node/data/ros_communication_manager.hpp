#pragma once

#include <rclcpp/node.hpp>

#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <custom_msgs/msg/offboard_ctrl.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/string.hpp>
#include <custom_msgs/msg/status_task.hpp>

#include <nlohmann/json.hpp>

#include "behavior_node/data/data_cache.hpp"
#include "behavior_node/data/mission_context.hpp"

class ROSCommunicationManager {
 private:
  rclcpp::Node::SharedPtr node_;

  // 发布器管理

  std::unordered_map<std::string, rclcpp::PublisherBase::SharedPtr> publishers_;

  // 订阅器管理
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;

  // 服务客户端管理
  std::unordered_map<std::string, rclcpp::ClientBase::SharedPtr> service_clients_;

  // 数据缓存引用
  std::shared_ptr<DataCache> data_cache_;
  std::shared_ptr<MissionContext> mission_context_;

 public:
  ROSCommunicationManager(rclcpp::Node::SharedPtr node, std::shared_ptr<DataCache> cache)
      : node_(std::move(node)), data_cache_(std::move(cache)) {}

  void initialize() {
    setupPublishers();
    setupSubscriptions();
    setupServiceClients();
  }

  // 类型安全的发布接口
  template<typename T>
  bool publish(const std::string &topic, const T &msg) {
    if (publishers_.count(topic) != 0) {
      if (auto publisher = std::dynamic_pointer_cast<rclcpp::Publisher<T>>(publishers_[topic])) {
        publisher->publish(msg);
        return true;
      }
    }
    return false;
  };

  // 判断服务的客户端是否就绪
  bool isServiceReady(const std::string &service_name) {
    if (service_clients_.count(service_name) > 0) {
      if (auto client = std::dynamic_pointer_cast<rclcpp::ClientBase>(service_clients_[service_name])) {
        return client->wait_for_service(std::chrono::milliseconds(50)) && client->service_is_ready();
      }
    }
    return false;
  }

  // 服务调用接口
  template<typename ServiceT>
  auto callService(const std::string &service_name, typename ServiceT::Request::SharedPtr request) {
    if (service_clients_.count(service_name) > 0) {
      if (auto client = std::dynamic_pointer_cast<rclcpp::Client<ServiceT>>(service_clients_[service_name])) {
        return client->async_send_request(request);
      }
    }
    return std::shared_future<typename ServiceT::Response::SharedPtr>{};
  };

 private:
  void setupPublishers();

  void setupSubscriptions();

  void setupServiceClients();

  void handleMissionJSON(std_msgs::msg::String::SharedPtr msg);

  bool validateStage(const nlohmann::json &stage);

  void processStartCommand(const nlohmann::json &stage);

  bool validateAction(const nlohmann::json &action);

  void processSelfMission(const nlohmann::json &action,
                          int stage_sn,
                          const std::vector<int> &group_members,
                          const nlohmann::json &all_actions);

  void processFormationOffsets(const nlohmann::json &actions, int group_id);

  void processStopCommand(const nlohmann::json &stage);

  void processInsertCommand(const nlohmann::json &stage);

  void updateExcludedGroupMembers(const std::set<uint8_t> &executing_ids);

  void publishTaskStatus(int stage_sn, StatusStage status);
};