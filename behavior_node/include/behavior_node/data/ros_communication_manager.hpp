#pragma once

#include <rclcpp/rclcpp.hpp>
#include <unordered_map>
#include <vector>
#include <nlohmann/json.hpp>

#include "behavior_node/data/data_cache.hpp"
#include "behavior_node/data/mission_context.hpp"
#include "behavior_node/core/types.hpp"
#include "behavior_node/core/message_queue.hpp"
#include "behavior_node/data/ros_interface_definitions.hpp"

// ROS消息类型
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <custom_msgs/msg/offboard_ctrl.hpp>
#include <custom_msgs/msg/status_task.hpp>
#include <custom_msgs/msg/simple_vehicle.hpp>
#include <custom_msgs/msg/object_computation.hpp>
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

namespace behavior_core {

// JSON任务参数结构
struct TaskAction {
  int groupid;
  int id;
  std::string name;
  std::unordered_map<std::string, nlohmann::json> params;
  std::unordered_map<std::string, nlohmann::json> triggers;
};

struct TaskStage {
  std::string name;
  int sn;
  std::string cmd;  // start, pause, continue, stop
  std::vector<TaskAction> actions;
};

struct TaskMission {
  std::vector<TaskStage> stages;
};

} // namespace behavior_core

class ROSCommunicationManager {
 private:
  rclcpp::Node::SharedPtr node_;

  std::unordered_map<std::string, rclcpp::PublisherBase::SharedPtr> publishers_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  std::unordered_map<std::string, rclcpp::ClientBase::SharedPtr> service_clients_;

  std::shared_ptr<Cache> data_cache_;
  std::shared_ptr<MissionContext> mission_context_;
  std::shared_ptr<behavior_core::MessageQueue> message_queue_;

  rclcpp::Logger logger_;

 public:
  ROSCommunicationManager(
      rclcpp::Node::SharedPtr node,
      std::shared_ptr<Cache> cache,
      std::shared_ptr<MissionContext> context,
      std::shared_ptr<behavior_core::MessageQueue> msg_queue)
      : node_(std::move(node)),
        data_cache_(std::move(cache)),
        mission_context_(std::move(context)),
        message_queue_(std::move(msg_queue)),
        logger_(node_->get_logger()) {

    txtLog().info(THISMODULE "Created ROS communication manager");
  }

  void initialize() {
    setupPublishers();
    setupSubscriptions();
    setupServiceClients();
    txtLog().info(THISMODULE "ROS Communication Manager initialized");
  }

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
  void publishOffboardControl(const custom_msgs::msg::OffboardCtrl& msg) {
    publish(ros_interface::topics::OFFBOARD_CONTROL, msg);
  }

  void publishCommandResponse(const custom_msgs::msg::CommandResponse& msg) {
    publish(ros_interface::topics::OUTER_RESPONSE, msg);
  }

  void publishTaskStatus(const custom_msgs::msg::StatusTask& msg) {
    publish(ros_interface::topics::OUTER_STATUS_TASK, msg);
  }

  void publishSystemStatus(const custom_msgs::msg::StatusTask& msg) {
    publishTaskStatus(msg);
  }

  // ================================ 服务调用接口 ================================
  bool isServiceReady(const std::string &service_name) {
    if (service_clients_.contains(service_name)) {
      if (auto client = std::dynamic_pointer_cast<rclcpp::ClientBase>(service_clients_.at(service_name))) {
        return client->wait_for_service(std::chrono::milliseconds(5)) &&
            client->service_is_ready();
      }
    } else {
      txtLog().error(THISMODULE "Service client not found for service: %s", service_name.c_str());
    }
    return false;
  }

  template<typename ServiceT>
  auto callService(const std::string &service_name,
                   typename ServiceT::Request::SharedPtr request) {
    if (service_clients_.contains(service_name)) {
      if (auto client = std::dynamic_pointer_cast<rclcpp::Client<ServiceT>>(service_clients_.at(service_name))) {
        try {
          auto result = client->async_send_request(request);
          return result;
        } catch (const std::exception &e) {
          txtLog().error(THISMODULE "Failed to call service %s: %s",
                         service_name.c_str(), e.what());
        }
      }
    } else {
      txtLog().error(THISMODULE "Service client not found for service: %s", service_name.c_str());
    }
    return std::shared_future<typename ServiceT::Response::SharedPtr>{};
  }

  // 飞行控制服务接口
  bool callTakeoffService(double altitude) {
    auto request = std::make_shared<custom_msgs::srv::CommandLong::Request>();
    request->command = 22; // MAV_CMD_NAV_TAKEOFF
    request->param7 = altitude;

    auto result = callService<custom_msgs::srv::CommandLong>(
        ros_interface::services::SET_FLIGHT_MODE, request);

    if (result.valid()) {
      auto response = result.get();
      return response && response->result == 0;
    }
    return false;
  }

  bool callLandService() {
    auto request = std::make_shared<custom_msgs::srv::CommandLong::Request>();
    request->command = 21; // MAV_CMD_NAV_LAND

    auto result = callService<custom_msgs::srv::CommandLong>(
        ros_interface::services::SET_FLIGHT_MODE, request);

    if (result.valid()) {
      auto response = result.get();
      return response && response->result == 0;
    }
    return false;
  }

  bool callRtlService() {
    auto request = std::make_shared<custom_msgs::srv::CommandLong::Request>();
    request->command = 20; // MAV_CMD_NAV_RETURN_TO_LAUNCH

    auto result = callService<custom_msgs::srv::CommandLong>(
        ros_interface::services::SET_FLIGHT_MODE, request);

    if (result.valid()) {
      auto response = result.get();
      return response && response->result == 0;
    }
    return false;
  }

  bool callAttackService(int target_id, int src_id, int dst_id) {
    auto request = std::make_shared<custom_msgs::srv::CommandInt::Request>();
    request->param1 = target_id;
    request->param2 = src_id;
    request->param3 = dst_id;

    auto result = callService<custom_msgs::srv::CommandInt>(
        ros_interface::services::GUIDANCE_SWITCH, request);

    if (result.valid()) {
      auto response = result.get();
      return response && response->success;
    }
    return false;
  }

  // ================================ 行为树控制接口 ================================
  void requestTreeLoad(const std::string &tree_name) {
    if (message_queue_) {
      behavior_core::BehaviorMessage msg(behavior_core::BehaviorCommand::LOAD_TREE, tree_name, {});
      message_queue_->push(msg);
      txtLog().info(THISMODULE "Requested to load tree: %s", tree_name.c_str());
    }
  }

  void requestTreeStop() {
    if (message_queue_) {
      behavior_core::BehaviorMessage msg(behavior_core::BehaviorCommand::STOP_TREE, "", {});
      message_queue_->push(msg);
      txtLog().info(THISMODULE "Requested to stop tree");
    }
  }

 private:
  // ================================ 发布器设置 ================================
  void setupPublishers() {
    using namespace ros_interface;

    try {
      // 图像分发发布
      publishers_[topics::INFO_IMAGE_DISTRIBUTE] =
          node_->create_publisher<custom_msgs::msg::ImageDistribute>(
              topics::INFO_IMAGE_DISTRIBUTE, rclcpp::QoS(10));

      // home点发布
      publishers_[topics::SET_COORD] =
          node_->create_publisher<geometry_msgs::msg::Point>(
              topics::SET_COORD, rclcpp::QoS(10));

      // offboard控制发布
      publishers_[topics::OFFBOARD_CONTROL] =
          node_->create_publisher<custom_msgs::msg::OffboardCtrl>(
              topics::OFFBOARD_CONTROL, rclcpp::SensorDataQoS());

      // 飞机类型设置发布
      publishers_[topics::SET_VEHICLE_TYPE] =
          node_->create_publisher<std_msgs::msg::UInt8>(
              topics::SET_VEHICLE_TYPE, rclcpp::QoS(1));

      // 外部响应发布
      publishers_[topics::OUTER_RESPONSE] =
          node_->create_publisher<custom_msgs::msg::CommandResponse>(
              topics::OUTER_RESPONSE, rclcpp::QoS(10));

      // 外部状态发布
      publishers_[topics::OUTER_STATUS_TASK] =
          node_->create_publisher<custom_msgs::msg::StatusTask>(
              topics::OUTER_STATUS_TASK, rclcpp::QoS(10));

      // 防撞距离发布
      publishers_[topics::SET_ANTI_COLLISION_DIS] =
          node_->create_publisher<std_msgs::msg::Float32>(
              topics::SET_ANTI_COLLISION_DIS, rclcpp::QoS(1));

      // 航线发布
      publishers_[topics::SET_NAVLINE] =
          node_->create_publisher<geometry_msgs::msg::Polygon>(
              topics::SET_NAVLINE, rclcpp::QoS(1));

      // 编队成员发布
      publishers_[topics::SET_GROUP_IDS] =
          node_->create_publisher<std_msgs::msg::UInt8MultiArray>(
              topics::SET_GROUP_IDS, rclcpp::QoS(1));

      // 编队速度发布
      publishers_[topics::SET_GROUP_SPEED] =
          node_->create_publisher<std_msgs::msg::Float32>(
              topics::SET_GROUP_SPEED, rclcpp::QoS(1));

      // 编队偏移发布
      publishers_[topics::SET_GROUP_OFFSET] =
          node_->create_publisher<geometry_msgs::msg::Polygon>(
              topics::SET_GROUP_OFFSET, rclcpp::QoS(1));

      // 到达距离发布
      publishers_[topics::SET_ARRIVAL_DIS] =
          node_->create_publisher<std_msgs::msg::Float32>(
              topics::SET_ARRIVAL_DIS, rclcpp::QoS(1));

      // 循环次数发布
      publishers_[topics::SET_LOOPS] =
          node_->create_publisher<std_msgs::msg::Int32>(
              topics::SET_LOOPS, rclcpp::QoS(1));

      // 编组设置发布
      publishers_[topics::SET_GROUP] =
          node_->create_publisher<std_msgs::msg::UInt8>(
              topics::SET_GROUP,rclcpp::QoS(1));

      // 编队队形发布
      publishers_[topics::SET_FORMATION] =
          node_->create_publisher<custom_msgs::msg::ParamShort>(
              topics::SET_FORMATION, rclcpp::QoS(1));

      // 手动控制发布
      publishers_[topics::MANUAL_CONTROL] =
          node_->create_publisher<mavros_msgs::msg::ManualControl>(
              topics::MANUAL_CONTROL, rclcpp::QoS(1));

      // 图像通道发布
      publishers_[topics::INFO_IMAGE_CHANNEL] =
          node_->create_publisher<std_msgs::msg::UInt8>(
              topics::INFO_IMAGE_CHANNEL, rclcpp::QoS(1));

      // 相机信息发布
      publishers_[topics::INFO_CAMERA_CONTROL] =
          node_->create_publisher<custom_msgs::msg::MultispectralCamCtrl>(
              topics::INFO_CAMERA_CONTROL, rclcpp::QoS(1));

      txtLog().info(THISMODULE "All publishers created successfully");

    } catch (const std::exception &e) {
      txtLog().error(THISMODULE "Failed to create publishers: %s", e.what());
    }
  }

  // ================================ 订阅器设置 ================================
  void setupSubscriptions() {
    using namespace ros_interface;

    try {
      // 任务控制订阅
      subscriptions_.push_back(
          node_->create_subscription<std_msgs::msg::String>(
              topics::STAGE_JSON,
              rclcpp::QoS(10),
              [this](std_msgs::msg::String::SharedPtr msg) {
                handleMissionJSON(msg);
              }
          )
      );

      // 命令订阅
      subscriptions_.push_back(
          node_->create_subscription<custom_msgs::msg::CommandRequest>(
              topics::COMMAND,
              rclcpp::QoS(10),
              [this](custom_msgs::msg::CommandRequest::SharedPtr msg) {
                handleCommand(msg);
              }
          )
      );

      // 目标跟踪打击订阅
      subscriptions_.push_back(
          node_->create_subscription<custom_msgs::msg::ObjectAttackDesignate>(
              topics::ATTACK_DESIGNATE,
              rclcpp::QoS(10),
              [this](custom_msgs::msg::ObjectAttackDesignate::SharedPtr msg) {
                handleAttackDesignate(msg);
              }
          )
      );

      // 飞机状态订阅
      subscriptions_.push_back(
          node_->create_subscription<custom_msgs::msg::SimpleVehicle>(
              topics::VEHICLE_STATE,
              rclcpp::SensorDataQoS(),
              [this](custom_msgs::msg::SimpleVehicle::SharedPtr msg) {
                if (data_cache_) {
                  data_cache_->updateVehicleState(msg);
                }
              }
          )
      );

      // 航路点距离信息订阅
      subscriptions_.push_back(
          node_->create_subscription<custom_msgs::msg::DisTarget>(
              topics::WAYPOINT,
              rclcpp::SensorDataQoS(),
              [this](custom_msgs::msg::DisTarget::SharedPtr msg) {
                if (auto id = mission_context_->getWpId();
                    (id == 0xFFFFFFFF || msg->id != id) && mission_context_->getArrivalDistance() > msg->dis / 1e3) {
                  mission_context_->setWpId(msg->id);
                  mission_context_->incLoopIndex();
                }
              }
          )
      );

      // 目标检测订阅
      subscriptions_.push_back(
          node_->create_subscription<custom_msgs::msg::ObjectComputation>(
              topics::OBJECT_DETECTION,
              rclcpp::SensorDataQoS(),
              [this](custom_msgs::msg::ObjectComputation::SharedPtr msg) {
                if (data_cache_) {
                  data_cache_->updateDetectedObjects(msg);
                }
              }
          )
      );

      txtLog().info(THISMODULE "All subscriptions created successfully");

    } catch (const std::exception &e) {
      txtLog().error(THISMODULE "Failed to create subscriptions: %s", e.what());
    }
  }

  // ================================ 服务客户端设置 ================================
  void setupServiceClients() {
    using namespace ros_interface;

    try {
      // RTSP URL获取服务
      service_clients_[services::INFO_RTSP_URL] =
          node_->create_client<custom_msgs::srv::CommandString>(
              services::INFO_RTSP_URL);

      // 飞行模式设置服务
      service_clients_[services::SET_FLIGHT_MODE] =
          node_->create_client<custom_msgs::srv::CommandLong>(
              services::SET_FLIGHT_MODE);

      // 锁定解锁服务
      service_clients_[services::LOCK_UNLOCK] =
          node_->create_client<custom_msgs::srv::CommandBool>(
              services::LOCK_UNLOCK);

      // 跟踪打击控制服务
      service_clients_[services::GUIDANCE_SWITCH] =
          node_->create_client<custom_msgs::srv::CommandInt>(
              services::GUIDANCE_SWITCH);

      // 编队控制服务
      service_clients_[services::FORMATION_SWITCH] =
          node_->create_client<custom_msgs::srv::CommandInt>(
              services::FORMATION_SWITCH);

      txtLog().info(THISMODULE "All service clients created successfully");

    } catch (const std::exception &e) {
      txtLog().error(THISMODULE "Failed to create service clients: %s", e.what());
    }
  }

  // ================================ 消息处理方法 ================================
  void handleMissionJSON(const std_msgs::msg::String::SharedPtr msg) {
    try {
      nlohmann::json mission_json = nlohmann::json::parse(msg->data);

      if (!mission_json.contains("stage")) {
        txtLog().warnning(THISMODULE "Mission JSON missing 'stage' field");
        return;
      }

      auto stage_info = mission_json["stage"];
      if (!stage_info.is_array() || stage_info.empty()) {
        txtLog().warnning(THISMODULE "Invalid stage info format");
        return;
      }

      // 处理每个阶段
      for (const auto &stage : stage_info) {
        if (!validateStage(stage)) {
          continue;
        }

        if (std::string cmd = stage["cmd"].get<std::string>();cmd == "start") {
          auto actions = stage["actions"];
          if (!actions.is_array() || actions.empty()) return;

          int stage_sn = stage["sn"].get<int>();

          for (const auto &action : actions) {
            if (!validateAction(action)) continue;
            if (action["id"].get<int>() == data_cache_->getVehicleId()) {
              processSelfMission(action, stage_sn, actions);
              break;
            }
          }
        }
      }

    } catch (const nlohmann::json::exception &e) {
      txtLog().error(THISMODULE "JSON parsing error: %s", e.what());
    } catch (const std::exception &e) {
      txtLog().error(THISMODULE "Error handling mission JSON: %s", e.what());
    }
  }

  void handleAttackDesignate(const custom_msgs::msg::ObjectAttackDesignate::SharedPtr msg) {
    if (!data_cache_ || !mission_context_) return;
    auto vehicle_id = data_cache_->getVehicleId();
    bool is_attack = false;
    for (auto id : msg.get()->ids) {
      if (id == vehicle_id) {
        is_attack = true;
        break;
      }
    }
    //它机跟踪打击任务(除去终止任务)时 重新规划分组及分组偏移
    if (!is_attack) {
      mission_context_->setExcludedIds(msg.get()->ids);
      return;
    }
    mission_context_->setAttackObjLoc(msg.get()->objs);
    mission_context_->setTraceAttackType(msg.get()->type);
  }

  void handleCommand(custom_msgs::msg::CommandRequest::SharedPtr msg) {
    if (!data_cache_ || !mission_context_) return;

    auto vehicle_id = data_cache_->getVehicleId();

    if (msg->type == CmdType::HeartBeat) return;

    if (msg->dst != vehicle_id && (msg->type == CmdType::Takeoff || msg->type == CmdType::Land ||
        msg->type == CmdType::Loiter || msg->type == CmdType::DoTask ||
        msg->type == CmdType::Joystick || msg->type == CmdType::DesignAttackObj)) {
      mission_context_->addExcludedId(msg->dst);
    }

    processCommand(msg);
  }

  static bool validateStage(const nlohmann::json &stage) {
    return stage.contains("name") && stage.contains("sn") &&
        stage.contains("cmd") && stage.contains("actions");
  }

  bool validateAction(const nlohmann::json &action) {
    return action.contains("name") && action.contains("id") &&
        action.contains("groupid");
  }

  void processSelfMission(const nlohmann::json &action, int stage_sn,
                          const nlohmann::json &all_actions) {
    if (!mission_context_) return;

    std::string action_name = action["name"].get<std::string>();
    int group_id = action["groupid"].get<int>();

    std::vector<uint8_t> group_members;
    for (const auto &act : all_actions) {
      if (act["groupid"].get<int>() == group_id) {
        group_members.push_back(act["id"].get<int>());
      }
    }

    mission_context_->setAction(action_name);
    mission_context_->setStage(stage_sn);
    mission_context_->setGroupId(group_id);
    mission_context_->setGroupMembers(group_members);

    if (action.contains("params") && action["params"].is_array()) {
      for (const auto &param : action["params"]) {
        if (param.contains("name") && param.contains("value")) {
          mission_context_->setParameter(
              param["name"].get<std::string>(), param["value"]);
        }
      }
    }

    if (action.contains("triggers") && action["triggers"].is_array()) {
      for (const auto &trigger : action["triggers"]) {
        if (trigger.contains("name") && trigger.contains("value")) {
          mission_context_->setTrigger(
              trigger["name"].get<std::string>(), trigger["value"]);
        }
      }
    }

    publishTaskStatus(stage_sn, StatusStage::StsNoStart);

    std::string tree_name = action_name;
    requestTreeLoad(tree_name);

    txtLog().info(THISMODULE "Started mission: %s, Stage: %d",
                  action_name.c_str(), stage_sn);
  }
    void processCommand(custom_msgs::msg::CommandRequest::SharedPtr msg) {
    custom_msgs::msg::CommandResponse response;
    response.id = data_cache_->getVehicleId();
    response.src = CtrlType::Cmd;
    response.type = msg->type;
    response.status = CmdStatus::Success;

    switch (msg->type) {
      case CmdType::SetVideo:handleSetVideoCommand(msg, response);
        break;
      case CmdType::CmdSetHome:handleSetHomeCommand(msg, response);
        break;
      case CmdType::Land:mission_context_->setAction("Land");
        break;
      default:txtLog().warnning(THISMODULE "Unhandled command type: %d", msg->type);
        break;
    }

    publish(ros_interface::topics::OUTER_RESPONSE, response);
  }
void handleSetVideoCommand(custom_msgs::msg::CommandRequest::SharedPtr msg,
                             custom_msgs::msg::CommandResponse &response) {
    custom_msgs::msg::ImageDistribute img_dis;
    img_dis.type = msg->param0;
    img_dis.rcvip = msg->param1;
    img_dis.rcvport = msg->param2;
    img_dis.resx = msg->param3;
    img_dis.resy = msg->param4;
    img_dis.fps = msg->fparam5;

    publish(ros_interface::topics::INFO_IMAGE_DISTRIBUTE, img_dis);

    if (msg->param0 == 2 && isServiceReady(ros_interface::services::INFO_RTSP_URL)) {
      auto request = std::make_shared<custom_msgs::srv::CommandString::Request>();
      auto future = callService<custom_msgs::srv::CommandString>(
          ros_interface::services::INFO_RTSP_URL, request);

      if (rclcpp::spin_until_future_complete(node_, future) ==
          rclcpp::FutureReturnCode::SUCCESS) {
        response.rslt = future.get()->rslt;
      }
    }
  }

  void handleSetHomeCommand(custom_msgs::msg::CommandRequest::SharedPtr msg,
                            custom_msgs::msg::CommandResponse &response) {
    if (!data_cache_ || !mission_context_) return;

    auto vehicle_state = data_cache_->getVehicleState();
    if (vehicle_state && vehicle_state->lock == LockState::UNLOCK) {
      txtLog().warnning(THISMODULE "Vehicle is locked, cannot set home");
      response.status = CmdStatus::Failed;
      response.rslt = "Vehicle is locked, cannot set home";
      return;
    }

    geometry_msgs::msg::Point home_point;
    home_point.x = msg->param1 / 1e7;
    home_point.y = msg->param2 / 1e7;
    home_point.z = msg->param3 / 1e3;

    publish(ros_interface::topics::SET_COORD, home_point);
    mission_context_->setHomePoint(home_point);

    txtLog().info(THISMODULE "Set home point: (%.6f, %.6f, %.2f)",
                  home_point.x, home_point.y, home_point.z);
  }

  void updateExcludedGroupMembers(const std::set<uint8_t> &excluded_ids) {
    if (!mission_context_) return;

    auto current_members = mission_context_->getGroupMembers();
    std::vector<uint8_t> updated_members;

    for (uint8_t member_id : current_members) {
      if (excluded_ids.find(member_id) == excluded_ids.end()) {
        updated_members.push_back(member_id);
      }
    }

    if (updated_members.size() != current_members.size()) {
      mission_context_->setExcludedIds(excluded_ids);
      txtLog().info(THISMODULE "Updated excluded group members");
    }
  }

  void publishTaskStatus(int stage_sn, StatusStage status) {
    if (!data_cache_) return;

    custom_msgs::msg::StatusTask status_msg;
    status_msg.stage = stage_sn;
    status_msg.id = data_cache_->getVehicleId();
    status_msg.status = static_cast<int>(status);

    publish(ros_interface::topics::OUTER_STATUS_TASK, status_msg);
  }
  void processTaskStage(const behavior_core::TaskStage& stage) {
    try {
      // 更新任务上下文
      mission_context_->setStage(stage.sn);

      txtLog().info(THISMODULE "Processing stage: %s (sn=%d, cmd=%s) with %zu actions",
                    stage.name.c_str(), stage.sn, stage.cmd.c_str(), stage.actions.size());

      // 处理每个动作
      for (const auto& action : stage.actions) {
        processTaskAction(action, stage.cmd);
      }

      // 发布任务状态
      publishStageStatus(stage, behavior_core::TaskStatus::ONGOING);

    } catch (const std::exception& e) {
      txtLog().error(THISMODULE "Failed to process task stage: %s", e.what());
      publishStageStatus(stage, behavior_core::TaskStatus::FAILED);
    }
  }

  void processTaskAction(const behavior_core::TaskAction& action, const std::string& cmd) {
    try {
      // 查找对应的行为树
      auto tree_it = task_to_tree_mapping_.find(action.name);
      if (tree_it == task_to_tree_mapping_.end()) {
        txtLog().warnning(THISMODULE "Unknown task action: %s", action.name.c_str());
        return;
      }

      // 构建行为树名称 (格式: TaskName-cmd)
      std::string tree_name = tree_it->second + "-" + cmd;

      // 将参数存储到任务上下文
      for (const auto& [key, value] : action.params) {
        mission_context_->setParameter(key, value);
      }

      // 创建行为树执行消息
      auto message = std::make_shared<behavior_core::TreeMessage>();
      message->type = behavior_core::MessageType::TREE_EXECUTION;
      message->tree_name = tree_name;
      message->action_name = action.name;
      message->group_id = action.groupid;
      message->vehicle_id = action.id;
      message->parameters = action.params;

      message_queue_->push(message);

      txtLog().info(THISMODULE "Queued tree execution: %s for vehicle %d (group %d)",
                    tree_name.c_str(), action.id, action.groupid);

    } catch (const std::exception& e) {
      txtLog().error(THISMODULE "Failed to process task action %s: %s",
                     action.name.c_str(), e.what());
    }
  }

  void publishStageStatus(const behavior_core::TaskStage& stage, behavior_core::TaskStatus status) {
    try {
      custom_msgs::msg::StatusTask status_msg;
      status_msg.id = data_cache_->getVehicleId();
      status_msg.stage = stage.sn;
      status_msg.status = static_cast<int>(status);
      publishTaskStatus(status_msg);

    } catch (const std::exception& e) {
      txtLog().error(THISMODULE "Failed to publish stage status: %s", e.what());
    }
  }
};