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

  // 行为树控制接口
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
  void setupPublishers() {
    using namespace ros_interface;

    try {
      // offboard控制发布
      publishers_[topics::OFFBOARD_CONTROL] =
          node_->create_publisher<custom_msgs::msg::OffboardCtrl>(
              topics::OFFBOARD_CONTROL, qos::control_commands());
      // 防撞距离发布
      publishers_[topics::SET_ANTI_COLLISION_DIS] =
          node_->create_publisher<std_msgs::msg::Float32>(
              topics::SET_ANTI_COLLISION_DIS, qos::control_commands());
      // 到达距离发布
      publishers_[topics::SET_ARRIVAL_DIS] =
          node_->create_publisher<std_msgs::msg::Float32>(
              topics::SET_ARRIVAL_DIS, qos::control_commands());
      // 编组设置发布
      publishers_[topics::SET_GROUP] =
          node_->create_publisher<std_msgs::msg::UInt8>(
              topics::SET_GROUP, qos::control_commands());
      // 飞机类型设置发布
      publishers_[topics::SET_VEHICLE_TYPE] =
          node_->create_publisher<std_msgs::msg::UInt8>(
              topics::SET_VEHICLE_TYPE, qos::control_commands());
      // 航线发布
      publishers_[topics::SET_NAVLINE] =
          node_->create_publisher<geometry_msgs::msg::Polygon>(
              topics::SET_NAVLINE, qos::mission_commands());
      // home点发布
      publishers_[topics::SET_COORD] =
          node_->create_publisher<geometry_msgs::msg::Point>(
              topics::SET_COORD, qos::control_commands());
      // 编队偏移发布
      publishers_[topics::SET_GROUP_OFFSET] =
          node_->create_publisher<geometry_msgs::msg::Polygon>(
              topics::SET_GROUP_OFFSET, qos::status_info());
      // 外部状态发布
      publishers_[topics::OUTER_STATUS_TASK] =
          node_->create_publisher<custom_msgs::msg::StatusTask>(
              topics::OUTER_STATUS_TASK, qos::status_info());
      // 手动控制发布
      publishers_[topics::MANUAL_CONTROL] =
          node_->create_publisher<mavros_msgs::msg::ManualControl>(
              topics::MANUAL_CONTROL, qos::control_commands());
      // 外部响应发布
      publishers_[topics::OUTER_RESPONSE] =
          node_->create_publisher<custom_msgs::msg::CommandResponse>(
              topics::OUTER_RESPONSE, qos::control_commands());
      // 图像分发发布
      publishers_[topics::INFO_IMAGE_DISTRIBUTE] =
          node_->create_publisher<std_msgs::msg::UInt8MultiArray>(
              topics::INFO_IMAGE_DISTRIBUTE, qos::status_info());
      // 图像通道发布
      publishers_[topics::INFO_IMAGE_CHANNEL] =
          node_->create_publisher<std_msgs::msg::UInt8>(
              topics::INFO_IMAGE_CHANNEL, qos::status_info());
      // 相机信息发布
      publishers_[topics::INFO_CAMERA_CONTROL] =
          node_->create_publisher<custom_msgs::msg::MultispectralCamCtrl>(
              topics::INFO_CAMERA_CONTROL, qos::status_info());
      // 编队队形发布
      publishers_[topics::SET_FORMATION] =
          node_->create_publisher<custom_msgs::msg::ParamShort>(
              topics::SET_FORMATION, qos::mission_commands());
      // 编组id发布
      publishers_[topics::SET_GROUP] =
          node_->create_publisher<std_msgs::msg::UInt8>(
              topics::SET_GROUP, qos::status_info());
      // 循环次数发布
      publishers_[topics::SET_LOOPS] =
          node_->create_publisher<std_msgs::msg::Int32>(
              topics::SET_LOOPS, qos::mission_commands());
      // 编队速度发布
      publishers_[topics::SET_SPEED] =
          node_->create_publisher<std_msgs::msg::Float32>(
              topics::SET_SPEED, qos::mission_commands());
      // 编队成员发布
      publishers_[topics::SET_GROUP_IDS] =
          node_->create_publisher<std_msgs::msg::UInt8MultiArray>(
              topics::SET_GROUP_IDS, qos::status_info());

      txtLog().info(THISMODULE "All publishers created successfully");

    } catch (const std::exception &e) {
      txtLog().error(THISMODULE "Failed to create publishers: %s", e.what());
    }
  }

  void setupSubscriptions() {
    using namespace ros_interface;

    try {
      // 任务控制订阅
      subscriptions_.push_back(
          node_->create_subscription<std_msgs::msg::String>(
              topics::STAGE_JSON,
              qos::mission_commands(),
              [this](std_msgs::msg::String::SharedPtr msg) {
                handleMissionJSON(msg);
              }
          )
      );

      // 命令订阅
      subscriptions_.push_back(
          node_->create_subscription<custom_msgs::msg::CommandRequest>(
              topics::COMMAND,
              qos::control_commands(),
              [this](custom_msgs::msg::CommandRequest::SharedPtr msg) {
                handleCommand(msg);
              }
          )
      );

      // 目标跟踪打击订阅
      subscriptions_.push_back(
          node_->create_subscription<custom_msgs::msg::ObjectAttackDesignate>(
              topics::ATTACK_DESIGNATE,
              qos::sensor_data(),
              [this](custom_msgs::msg::ObjectAttackDesignate::SharedPtr msg) {
                handleAttackDesignate(msg);
              }
          )
      );

      // 飞机状态订阅
      subscriptions_.push_back(
          node_->create_subscription<custom_msgs::msg::SimpleVehicle>(
              topics::VEHICLE_STATE,
              qos::mission_commands(),
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
              qos::sensor_data(),
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
              qos::sensor_data(),
              [this](custom_msgs::msg::ObjectComputation::SharedPtr msg) {
                if (data_cache_) {
                  data_cache_->updateDetectedObjects(msg);
                }
              }
          )
      );

      // 摇杆输入订阅
      subscriptions_.push_back(
          node_->create_subscription<sensor_msgs::msg::Joy>(
              topics::JOYSTICK,
              qos::sensor_data(),
              [this](sensor_msgs::msg::Joy::SharedPtr msg) {
                handleJoystickInput(msg);
              }
          )
      );

      // 任务阶段订阅
      subscriptions_.push_back(
          node_->create_subscription<custom_msgs::msg::TaskStage>(
              topics::OUTER_TASK_STAGE,
              qos::mission_commands(),
              [this](custom_msgs::msg::TaskStage::SharedPtr msg) {
                handleTaskStage(msg);
              }
          )
      );

      txtLog().info(THISMODULE "All subscriptions created successfully");

    } catch (const std::exception &e) {
      txtLog().error(THISMODULE "Failed to create subscriptions: %s", e.what());
    }
  }

  void setupServiceClients() {
    using namespace ros_interface;

    try {
      service_clients_[services::INFO_RTSP_URL] =
          node_->create_client<custom_msgs::srv::CommandString>(
              services::INFO_RTSP_URL);

      service_clients_[services::SET_FLIGHT_MODE] =
          node_->create_client<custom_msgs::srv::CommandLong>(
              services::SET_FLIGHT_MODE);

      service_clients_[services::LOCK_UNLOCK] =
          node_->create_client<custom_msgs::srv::CommandBool>(
              services::LOCK_UNLOCK);

      service_clients_[services::GUIDANCE_SWITCH] =
          node_->create_client<custom_msgs::srv::CommandBool>(
              services::GUIDANCE_SWITCH);

      service_clients_[services::FORMATION_SWITCH] =
          node_->create_client<custom_msgs::srv::CommandInt>(
              services::FORMATION_SWITCH);

      txtLog().info(THISMODULE "All service clients created successfully");

    } catch (const std::exception &e) {
      txtLog().error(THISMODULE "Failed to create service clients: %s", e.what());
    }
  }

  void handleMissionJSON(std_msgs::msg::String::SharedPtr msg) {
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

  void handleAttackDesignate(custom_msgs::msg::ObjectAttackDesignate::SharedPtr msg) {
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

  void handleTaskStage(custom_msgs::msg::TaskStage::SharedPtr msg) {
    if (!data_cache_ || !mission_context_) return;
    mission_context_->setTaskStage(*msg);

    auto vehicle_id = data_cache_->getVehicleId();
    auto ids = msg->head.ids;

    if (std::find(ids.begin(), ids.end(), vehicle_id) != ids.end()) {
      if (ids.size() == msg->formoffset.points.size()) {
        mission_context_->setGroupMembers(ids);
        mission_context_->setOffsets(msg->formoffset);

        auto set_type = mission_context_->getSetType();
        if (set_type & SetContentType::WAY_PTS) {
          publish(ros_interface::topics::SET_NAVLINE, msg->formoffset);
        } else if (set_type & SetContentType::LOOPS) {
          publish(ros_interface::topics::SET_LOOPS, msg->formoffset);
        }
        txtLog().info(THISMODULE "Updated group members and offsets");
      }
    } else {
      updateExcludedGroupMembers({ids.begin(), ids.end()});
    }
  }

  void handleJoystickInput(sensor_msgs::msg::Joy::SharedPtr msg) {
    if (!data_cache_) return;

    data_cache_->updateJoyControl(msg);

    mavros_msgs::msg::ManualControl manual_control_msg;
    if (msg->buttons.size() >= 4) {
      manual_control_msg.x = msg->buttons[0];
      manual_control_msg.y = msg->buttons[1];
      manual_control_msg.z = msg->buttons[2];
      manual_control_msg.r = msg->buttons[3];
      publish(ros_interface::topics::MANUAL_CONTROL, manual_control_msg);
    }
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
    home_point.x = msg->param0 / 1e7;
    home_point.y = msg->param1 / 1e7;
    home_point.z = msg->param2 / 1e3;

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
};