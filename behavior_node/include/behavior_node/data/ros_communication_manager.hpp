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
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <custom_msgs/msg/offboard_ctrl.hpp>
#include <custom_msgs/msg/status_task.hpp>
#include <custom_msgs/msg/simple_vehicle.hpp>
#include <custom_msgs/msg/object_computation.hpp>
#include <custom_msgs/msg/command_request.hpp>
#include <custom_msgs/msg/command_response.hpp>
#include <custom_msgs/msg/multispectral_cam_ctrl.hpp>
#include <custom_msgs/msg/task_stage.hpp>
#include <custom_msgs/msg/image_distribute.hpp>
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
    auto it = publishers_.find(topic);
    if (it != publishers_.end()) {
      if (auto publisher = std::dynamic_pointer_cast<rclcpp::Publisher<T>>(it->second)) {
        try {
          publisher->publish(msg);
          return true;
        } catch (const std::exception &e) {
          txtLog().error(THISMODULE "Failed to publish to %s: %s", topic.c_str(), e.what());
        }
      }
    }
    return false;
  }

  bool isServiceReady(const std::string &service_name) {
    auto it = service_clients_.find(service_name);
    if (it != service_clients_.end()) {
      if (auto client = std::dynamic_pointer_cast<rclcpp::ClientBase>(it->second)) {
        return client->wait_for_service(std::chrono::milliseconds(50)) &&
            client->service_is_ready();
      }
    }
    return false;
  }

  template<typename ServiceT>
  auto callService(const std::string &service_name,
                   typename ServiceT::Request::SharedPtr request) {
    auto it = service_clients_.find(service_name);
    if (it != service_clients_.end()) {
      if (auto client = std::dynamic_pointer_cast<rclcpp::Client<ServiceT>>(it->second)) {
        try {
          return client->async_send_request(request);
        } catch (const std::exception &e) {
          txtLog().error(THISMODULE "Failed to call service %s: %s",
                       service_name.c_str(), e.what());
        }
      }
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
      publishers_[topics::OFFBOARD_CONTROL] =
          node_->create_publisher<custom_msgs::msg::OffboardCtrl>(
              topics::OFFBOARD_CONTROL, qos::control_commands());

      publishers_[topics::SET_ANTI_COLLISION_DIS] =
          node_->create_publisher<std_msgs::msg::Float32>(
              topics::SET_ANTI_COLLISION_DIS, qos::control_commands());

      publishers_[topics::SET_ARRIVAL_DIS] =
          node_->create_publisher<std_msgs::msg::Float32>(
              topics::SET_ARRIVAL_DIS, qos::control_commands());

      publishers_[topics::SET_SPEED] =
          node_->create_publisher<std_msgs::msg::Float32>(
              topics::SET_SPEED, qos::control_commands());

      publishers_[topics::SET_GROUP] =
          node_->create_publisher<std_msgs::msg::UInt8>(
              topics::SET_GROUP, qos::control_commands());

      publishers_[topics::SET_VEHICLE_TYPE] =
          node_->create_publisher<std_msgs::msg::UInt8>(
              topics::SET_VEHICLE_TYPE, qos::control_commands());

      publishers_[topics::SET_NAVLINE] =
          node_->create_publisher<geometry_msgs::msg::Polygon>(
              topics::SET_NAVLINE, qos::mission_commands());

      publishers_[topics::SET_COORD] =
          node_->create_publisher<geometry_msgs::msg::Point>(
              topics::SET_COORD, qos::control_commands());

      publishers_[topics::INFO_GROUP_OFFSET] =
          node_->create_publisher<geometry_msgs::msg::Polygon>(
              topics::INFO_GROUP_OFFSET, qos::status_info());

      publishers_[topics::OUTER_STATUS_TASK] =
          node_->create_publisher<custom_msgs::msg::StatusTask>(
              topics::OUTER_STATUS_TASK, qos::status_info());

      publishers_[topics::MANUAL_CONTROL] =
          node_->create_publisher<mavros_msgs::msg::ManualControl>(
              topics::MANUAL_CONTROL, qos::control_commands());

      publishers_[topics::OUTER_RESPONSE] =
          node_->create_publisher<custom_msgs::msg::CommandRequest>(
              topics::OUTER_RESPONSE, qos::control_commands());

      publishers_[topics::INFO_IMAGE_DISTRIBUTE] =
          node_->create_publisher<std_msgs::msg::UInt8MultiArray>(
              topics::INFO_IMAGE_DISTRIBUTE, qos::status_info());

      publishers_[topics::INFO_IMAGE_CHANNEL] =
          node_->create_publisher<std_msgs::msg::UInt8>(
              topics::INFO_IMAGE_CHANNEL, qos::status_info());

      publishers_[topics::INFO_CAMERA_CONTROL] =
          node_->create_publisher<custom_msgs::msg::MultispectralCamCtrl>(
              topics::INFO_CAMERA_CONTROL, qos::status_info());

      txtLog().info(THISMODULE "All publishers created successfully");

    } catch (const std::exception &e) {
      txtLog().error(THISMODULE "Failed to create publishers: %s", e.what());
    }
  }

  void setupSubscriptions() {
    using namespace ros_interface;

    try {
      // 飞机状态订阅
      subscriptions_.push_back(
          node_->create_subscription<custom_msgs::msg::SimpleVehicle>(
              topics::INFO_VEHICLE_STATE,
              qos::mission_commands(),
              [this](custom_msgs::msg::SimpleVehicle::SharedPtr msg) {
                if (data_cache_) {
                  data_cache_->updateVehicleState(msg);
                }
              }
          )
      );

      // 目标检测订阅
      subscriptions_.push_back(
          node_->create_subscription<custom_msgs::msg::ObjectComputation>(
              topics::INFO_OBJECT_DETECTION,
              qos::sensor_data(),
              [this](custom_msgs::msg::ObjectComputation::SharedPtr msg) {
                if (data_cache_) {
                  data_cache_->updateDetectedObjects(msg);
                }
              }
          )
      );

      // 任务控制订阅
      subscriptions_.push_back(
          node_->create_subscription<std_msgs::msg::String>(
              topics::OUTER_STAGE_JSON,
              qos::mission_commands(),
              [this](std_msgs::msg::String::SharedPtr msg) {
                handleMissionJSON(msg);
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

      // 命令订阅
      subscriptions_.push_back(
          node_->create_subscription<custom_msgs::msg::CommandRequest>(
              topics::OUTER_COMMAND,
              qos::control_commands(),
              [this](custom_msgs::msg::CommandRequest::SharedPtr msg) {
                handleCommand(msg);
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
      service_clients_[services::SET_FLIGHT_MODE] =
          node_->create_client<custom_msgs::srv::CommandLong>(
              services::SET_FLIGHT_MODE);

      service_clients_[services::LOCK_UNLOCK] =
          node_->create_client<custom_msgs::srv::CommandBool>(
              services::LOCK_UNLOCK);

      service_clients_[services::FORMATION_SWITCH] =
          node_->create_client<custom_msgs::srv::CommandInt>(
              services::FORMATION_SWITCH);

      service_clients_[services::INFO_RTSP_URL] =
          node_->create_client<custom_msgs::srv::CommandString>(
              services::INFO_RTSP_URL);

      txtLog().info(THISMODULE "All service clients created successfully");

    } catch (const std::exception &e) {
      txtLog().error(THISMODULE "Failed to create service clients: %s", e.what());
    }
  }

  void handleMissionJSON(std_msgs::msg::String::SharedPtr msg) {
    try {
      nlohmann::json mission_json = nlohmann::json::parse(msg->data);

      if (!mission_json.contains("stage")) {
        RCLCPP_WARN(logger_, "Mission JSON missing 'stage' field");
        return;
      }

      auto stage_info = mission_json["stage"];
      if (!stage_info.is_array() || stage_info.empty()) {
        RCLCPP_WARN(logger_, "Invalid stage info format");
        return;
      }

      // 处理每个阶段
      for (const auto &stage : stage_info) {
        if (!validateStage(stage)) {
          continue;
        }

        std::string cmd = stage["cmd"].get<std::string>();

        if (cmd == "set" || cmd == "start") {
          processStartCommand(stage);
        } else if (cmd == "del" || cmd == "stop") {
          processStopCommand(stage);
        } else if (cmd == "ins") {
          processInsertCommand(stage);
        }
      }

    } catch (const nlohmann::json::exception &e) {
      txtLog().error(THISMODULE "JSON parsing error: %s", e.what());
    } catch (const std::exception &e) {
      txtLog().error(THISMODULE "Error handling mission JSON: %s", e.what());
    }
  }

  void handleTaskStage(custom_msgs::msg::TaskStage::SharedPtr msg) {
    if (!data_cache_ || !mission_context_) return;

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

    if (msg->dst != vehicle_id) {
      if (msg->type == CmdType::Takeoff || msg->type == CmdType::Land ||
          msg->type == CmdType::Loiter || msg->type == CmdType::DoTask ||
          msg->type == CmdType::Joystick || msg->type == CmdType::DesignAttackObj) {
        mission_context_->addExcludedId(msg->dst);
      }
      return;
    }

    processCommand(msg);
  }

  bool validateStage(const nlohmann::json &stage) {
    return stage.contains("name") && stage.contains("sn") &&
        stage.contains("cmd") && stage.contains("actions");
  }

  void processStartCommand(const nlohmann::json &stage) {
    auto actions = stage["actions"];
    if (!actions.is_array() || actions.empty()) return;

    int stage_sn = stage["sn"].get<int>();
    auto vehicle_id = data_cache_->getVehicleId();

    for (const auto &action : actions) {
      if (!validateAction(action)) continue;

      int id = action["id"].get<int>();
      if (id == vehicle_id) {
        processSelfMission(action, stage_sn, actions);
        break;
      }
    }
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

    std::string tree_name = action_name + "_start";
    requestTreeLoad(tree_name);

    txtLog().info(THISMODULE "Started mission: %s, Stage: %d",
                action_name.c_str(), stage_sn);
  }

  void processStopCommand(const nlohmann::json &stage) {
    if (!mission_context_) return;

    int stage_sn = stage["sn"].get<int>();

    if (mission_context_->getStage() == stage_sn) {
      requestTreeStop();
      publishTaskStatus(stage_sn, StatusStage::StsFailed);
      mission_context_->clear();
      txtLog().info(THISMODULE "Stopped mission for stage: %d", stage_sn);
    }
  }

  void processInsertCommand(const nlohmann::json &stage) {
    txtLog().info(THISMODULE "Insert command received for stage: %s",
                stage["name"].get<std::string>().c_str());
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
      default:RCLCPP_WARN(logger_, "Unhandled command type: %d", msg->type);
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
      RCLCPP_WARN(logger_, "Vehicle is locked, cannot set home");
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

  void updateExcludedGroupMembers(const std::set<uint8_t> &executing_ids) {
    if (!mission_context_) return;

    auto current_members = mission_context_->getGroupMembers();
    std::vector<uint8_t> updated_members;

    for (uint8_t member_id : current_members) {
      if (executing_ids.find(member_id) == executing_ids.end()) {
        updated_members.push_back(member_id);
      }
    }

    if (updated_members.size() != current_members.size()) {
      mission_context_->setExcludedIds(executing_ids);
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