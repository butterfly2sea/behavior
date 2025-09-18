#pragma once

#include <rclcpp/rclcpp.hpp>
#include <unordered_map>
#include <vector>
#include <functional>
#include <nlohmann/json.hpp>

#include "behavior_node/data/data_cache.hpp"
#include "behavior_node/data/mission_context.hpp"
#include "behavior_node/core/types.hpp"
#include "behavior_node/core/message_queue.hpp"
#include "behavior_node/data/ros_interface_definitions.hpp"
#include "behavior_node/data/base_enum.hpp"

// ROS消息类型
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/uint8.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/uint8_multi_array.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <custom_msgs/msg/offboard_ctrl.hpp>
#include <custom_msgs/msg/status_task.hpp>
#include <custom_msgs/msg/simple_vehicle.hpp>
#include <custom_msgs/msg/object_computation.hpp>
#include <custom_msgs/msg/object_location.hpp>
#include <custom_msgs/msg/object_attack_designate.hpp>
#include <custom_msgs/msg/command_request.hpp>
#include <custom_msgs/msg/command_response.hpp>
#include <custom_msgs/msg/multispectral_cam_ctrl.hpp>
#include <custom_msgs/msg/task_stage.hpp>
#include <custom_msgs/msg/image_distribute.hpp>
#include <custom_msgs/msg/dis_target.hpp>
#include <mavros_msgs/msg/manual_control.hpp>

// ROS服务类型
#include <custom_msgs/srv/command_bool.hpp>
#include <custom_msgs/srv/command_int.hpp>
#include <custom_msgs/srv/command_long.hpp>
#include <custom_msgs/srv/command_string.hpp>

#include <log/Logger.hpp>

/**
 * 地面站指令结构（简化的JSON格式）
 */
struct GroundStationCommand {
  std::string cmd;        // 指令类型: start, pause, continue, stop, set, emergency
  std::string action;     // 动作名称: TakeOff, Land, Navline, PatternSearch, 等
  uint8_t vehicle_id;     // 目标飞机ID (0xFF: 全部飞机)
  uint8_t group_id;       // 目标分组ID (0: 分组无效)
  std::unordered_map<std::string, nlohmann::json> params;  // 参数
  std::string timestamp;  // 时间戳
};

/**
 * ROS通信管理器 - 统一管理所有ROS通信接口
 */
class ROSCommunicationManager {
 private:
  rclcpp::Node::SharedPtr node_;

  // 发布器管理
  std::unordered_map<std::string, rclcpp::PublisherBase::SharedPtr> publishers_;

  // 订阅器管理
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;

  // 服务客户端管理
  std::unordered_map<std::string, rclcpp::ClientBase::SharedPtr> service_clients_;

  // 依赖管理
  std::shared_ptr<Cache> data_cache_;
  std::shared_ptr<MissionContext> mission_context_;
  std::shared_ptr<behavior_core::MessageQueue> message_queue_;

  // 日志
  rclcpp::Logger logger_;

 public:
  ROSCommunicationManager(
      rclcpp::Node::SharedPtr node,
      std::shared_ptr<Cache> cache,
      std::shared_ptr<MissionContext> context,
      std::shared_ptr<behavior_core::MessageQueue> msg_queue);

  ~ROSCommunicationManager() = default;

  // 初始化所有ROS接口
  void initialize();

  // ================================ 发布接口 ================================

  template<typename T>
  bool publish(const std::string &topic, const T &msg) {
    if (publishers_.contains(topic)) {
      if (auto publisher = std::dynamic_pointer_cast<rclcpp::Publisher<T>>(publishers_.at(topic))) {
        try {
          publisher->publish(msg);
          return true;
        } catch (const std::exception &e) {
          txtLog().error(THISMODULE "Failed to publish to %s: %s", topic.c_str(), e.what());
        }
      } else {
        txtLog().error(THISMODULE "Publisher type mismatch for topic: %s", topic.c_str());
      }
    } else {
      txtLog().error(THISMODULE "Publisher not found for topic: %s", topic.c_str());
    }
    return false;
  }

  // 专用发布接口
  void publishOffboardControl(const custom_msgs::msg::OffboardCtrl& msg);
  void publishCommandResponse(const custom_msgs::msg::CommandResponse& msg);
  void publishTaskStatus(const custom_msgs::msg::StatusTask& msg);
  void publishImageDistribute(const custom_msgs::msg::ImageDistribute& msg);
  void publishCoordinate(const geometry_msgs::msg::Point& point);
  void publishVehicleType(uint8_t type);
  void publishAntiCollisionDistance(float distance);
  void publishNavline(const geometry_msgs::msg::Polygon& waypoints);
  void publishGroupIds(const std::vector<uint8_t>& ids);
  void publishGroupSpeed(float speed);
  void publishGroupOffset(const geometry_msgs::msg::Polygon& offsets);
  void publishGroupLoops(uint32_t loops);
  void publishPointTag(uint8_t tag);
  void publishGroundStationStatus(const std::string& status);

  // ================================ 服务调用接口 ================================

  bool isServiceReady(const std::string &service_name);

  // 通用服务调用
  template<typename ServiceT>
  std::future<typename ServiceT::Response::SharedPtr> callService(
      const std::string& service_name,
      typename ServiceT::Request::SharedPtr request) {

    if (service_clients_.contains(service_name)) {
      if (auto client = std::dynamic_pointer_cast<rclcpp::Client<ServiceT>>(
          service_clients_.at(service_name))) {

        if (!client->wait_for_service(std::chrono::seconds(2))) {
          txtLog().warning(THISMODULE "Service %s not available", service_name.c_str());

          // 创建一个失败的future
          std::promise<typename ServiceT::Response::SharedPtr> promise;
          promise.set_value(nullptr);
          return promise.get_future();
        }

        return client->async_send_request(request);
      }
    }

    txtLog().error(THISMODULE "Service client not found: %s", service_name.c_str());
    std::promise<typename ServiceT::Response::SharedPtr> promise;
    promise.set_value(nullptr);
    return promise.get_future();
  }

  // 专用服务调用接口
  std::future<custom_msgs::srv::CommandBool::Response::SharedPtr>
  callLockControl(bool lock);

  std::future<custom_msgs::srv::CommandLong::Response::SharedPtr>
  callFlightModeControl(uint8_t mode, float param7 = 0.0f);

  std::future<custom_msgs::srv::CommandLong::Response::SharedPtr>
  callTakeoffControl(float altitude);

  std::future<custom_msgs::srv::CommandLong::Response::SharedPtr>
  callLandControl();

  std::future<custom_msgs::srv::CommandString::Response::SharedPtr>
  callNavigationControl(const std::string& command);

  std::future<custom_msgs::srv::CommandLong::Response::SharedPtr>
  callTraceAttackControl(uint8_t frame, uint8_t command);

  std::future<custom_msgs::srv::CommandInt::Response::SharedPtr>
  callFormationSwitch(int formation_type);

  // ================================ 行为树控制接口 ================================

  void requestTreeLoad(const std::string &tree_name);
  void requestTreeStop();
  void requestTreePause();
  void requestTreeResume();
  void requestEmergencyStop();

 private:
  // ================================ 发布器设置 ================================
  void setupPublishers();

  // ================================ 订阅器设置 ================================
  void setupSubscriptions();

  // ================================ 服务客户端设置 ================================
  void setupServiceClients();

  // ================================ 消息处理方法 ================================

  // 外部指令处理
  void handleCommandRequest(const custom_msgs::msg::CommandRequest::SharedPtr msg);
  void handleGroundStationCommand(const std_msgs::msg::String::SharedPtr msg);

  // 状态信息处理
  void handleSimpleVehicle(const custom_msgs::msg::SimpleVehicle::SharedPtr msg);
  void handleOtherVehicleInfo(const custom_msgs::msg::SimpleVehicle::SharedPtr msg);
  void handleObjectComputation(const custom_msgs::msg::ObjectComputation::SharedPtr msg);
  void handleObjectLocation(const custom_msgs::msg::ObjectLocation::SharedPtr msg);
  void handleDisTarget(const custom_msgs::msg::DisTarget::SharedPtr msg);
  void handleTaskStage(const custom_msgs::msg::TaskStage::SharedPtr msg);

  // 控制输入处理
  void handleJoystick(const sensor_msgs::msg::Joy::SharedPtr msg);
  void handleManualControl(const mavros_msgs::msg::ManualControl::SharedPtr msg);
  void handleAttackDesignate(const custom_msgs::msg::ObjectAttackDesignate::SharedPtr msg);
  void handleCameraControl(const custom_msgs::msg::MultispectralCamCtrl::SharedPtr msg);

  // ================================ 辅助方法 ================================

  // 地面站指令解析
  static GroundStationCommand parseGroundStationCommand(const std::string& json_str);
  static bool validateGroundStationCommand(const GroundStationCommand& cmd);
  void processGroundStationCommand(const GroundStationCommand& cmd);

  // 响应生成
  void sendCommandResponse(const std::string& result, const std::string& message = "");
  void sendTaskStatusUpdate(StatusStage status, const std::string& description = "");

  // 错误处理
  void handleError(const std::string& operation, const std::exception& e);

  // 状态检查
  bool isCommandAllowed(const std::string& cmd) const;
  bool isVehicleTargeted(uint8_t vehicle_id, uint8_t group_id) const;
};