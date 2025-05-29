#include "behavior_node/data/ros_communication_manager.hpp"

#include <set>

#include <log/Logger.hpp>

#include <geometry_msgs/msg/polygon.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <custom_msgs/msg/simple_vehicle.hpp>
#include <custom_msgs/msg/offboard_ctrl.hpp>
#include <custom_msgs/msg/status_task.hpp>
#include <custom_msgs/msg/object_computation.hpp>
#include <custom_msgs/msg/command_request.hpp>
#include <custom_msgs/msg/command_response.hpp>
#include <mavros_msgs/msg/manual_control.hpp>
#include <custom_msgs/msg/image_distribute.hpp>
#include <custom_msgs/srv/command_bool.hpp>
#include <custom_msgs/srv/command_int.hpp>
#include <custom_msgs/srv/command_long.hpp>
#include <custom_msgs/srv/command_string.hpp>
#include <custom_msgs/msg/multispectral_cam_ctrl.hpp>
#include <custom_msgs/msg/task_stage.hpp>

#include <nlohmann/json.hpp>

#include "behavior_node/data/ros_interface_definitions.hpp"

void ROSCommunicationManager::setupPublishers() {
  using namespace ros_interface;

  // 控制命令发布器
  publishers_[topics::OFFBOARD_CONTROL] =
      node_->create_publisher<custom_msgs::msg::OffboardCtrl>(topics::OFFBOARD_CONTROL, qos::control_commands());

  // 防撞距离发布器
  publishers_[topics::SET_ANTI_COLLISION_DIS] =
      node_->create_publisher<std_msgs::msg::Float32>(topics::SET_ANTI_COLLISION_DIS, qos::control_commands());

  // 到达判定距离发布器
  publishers_[topics::SET_ARRIVAL_DIS] =
      node_->create_publisher<std_msgs::msg::Float32>(topics::SET_ARRIVAL_DIS, qos::control_commands());

  // 速度设置发布器
  publishers_[topics::SET_SPEED] =
      node_->create_publisher<std_msgs::msg::Float32>(topics::SET_SPEED, qos::control_commands());

  // 编组id发布器
  publishers_[topics::SET_GROUP] =
      node_->create_publisher<std_msgs::msg::UInt8>(topics::SET_GROUP, qos::control_commands());

  // 编队类型发布器
  publishers_[topics::SET_VEHICLE_TYPE] =
      node_->create_publisher<std_msgs::msg::UInt8>(topics::SET_VEHICLE_TYPE, qos::control_commands());

  // 航线 发布器组
  publishers_[topics::SET_NAVLINE] =
      node_->create_publisher<geometry_msgs::msg::Polygon>(topics::SET_NAVLINE, qos::mission_commands());

  // 编队偏移量信息发布器
  publishers_[topics::INFO_GROUP_OFFSET] =
      node_->create_publisher<geometry_msgs::msg::Polygon>(topics::INFO_GROUP_OFFSET, qos::status_info());

  // 状态发布器
  publishers_[topics::OUTER_STATUS_TASK] =
      node_->create_publisher<custom_msgs::msg::StatusTask>(topics::OUTER_STATUS_TASK, qos::status_info());

  // 手动控制命令发布器
  publishers_[topics::MANUAL_CONTROL] =
      node_->create_publisher<mavros_msgs::msg::ManualControl>(topics::MANUAL_CONTROL, qos::control_commands());

  // 命令响应发布器
  publishers_[topics::OUTER_RESPONSE] =
      node_->create_publisher<custom_msgs::msg::CommandRequest>(topics::OUTER_RESPONSE, qos::control_commands());

  // 图像分辨率发布器
  publishers_[topics::INFO_IMAGE_DISTRIBUTE] =
      node_->create_publisher<std_msgs::msg::UInt8MultiArray>(topics::INFO_IMAGE_DISTRIBUTE, qos::status_info());

  // 图像通道信息发布器
  publishers_[topics::INFO_IMAGE_CHANNEL] =
      node_->create_publisher<std_msgs::msg::UInt8>(topics::INFO_IMAGE_CHANNEL, qos::status_info());

  // 多光谱相机控制发布器
  publishers_[topics::INFO_CAMERA_CONTROL] =
      node_->create_publisher<custom_msgs::msg::MultispectralCamCtrl>(topics::INFO_CAMERA_CONTROL, qos::status_info());

  // home点发布器
  publishers_[topics::SET_COORD] =
      node_->create_publisher<geometry_msgs::msg::Point>(topics::SET_COORD, qos::control_commands());

}

void ROSCommunicationManager::setupSubscriptions() {
  using namespace ros_interface;

  // 飞机状态订阅
  subscriptions_.push_back(
      node_->create_subscription<custom_msgs::msg::SimpleVehicle>(
          topics::INFO_VEHICLE_STATE,
          qos::mission_commands(),
          [this](custom_msgs::msg::SimpleVehicle::SharedPtr msg) {
            data_cache_->updateVehicleState(msg);
          }
      )
  );

  // 目标检测订阅
  subscriptions_.push_back(
      node_->create_subscription<custom_msgs::msg::ObjectComputation>(
          topics::INFO_OBJECT_DETECTION,
          qos::sensor_data(),
          [this](custom_msgs::msg::ObjectComputation::SharedPtr msg) {
            data_cache_->updateDetectedObjects(msg);
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
            // 重置编队偏移量
            if (auto ids = msg->head.ids;
                std::find(ids.begin(), ids.end(), data_cache_->getVehicleId()) != ids.end()) {//如本机处于分组中则更新分组及偏移
              if (ids.size() == msg->formoffset.points.size()) {
                mission_context_->setGroupMembers(ids);
                mission_context_->setOffsets(msg->formoffset);

                if (mission_context_->getSetType() & SetContentType::WAY_PTS) {
                  publish(topics::SET_NAVLINE, msg->formoffset);
                } else if (mission_context_->getSetType() & SetContentType::LOOPS) {
                  publish(topics::SET_LOOPS, msg->formoffset);
                }
                txtLog().info(THISMODULE "update ids and offsets");
              }
            } else {//如本机不处于分组中则从原来分组中排除此分组飞机
              updateExcludedGroupMembers({ids.begin(), ids.end()});
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
            mavros_msgs::msg::ManualControl manual_control_msg;
            // TODO:: 此处的下标原由端口输入，但未找到默认值，待确认
            manual_control_msg.x = msg->buttons[0];
            manual_control_msg.y = msg->buttons[1];
            manual_control_msg.z = msg->buttons[2];
            manual_control_msg.r = msg->buttons[3];
            publish(topics::MANUAL_CONTROL, manual_control_msg);
            data_cache_->updateJoyControl(msg);
          }
      )
  );

// 命令订阅
  subscriptions_.
      push_back(
      node_->create_subscription<custom_msgs::msg::CommandRequest>(
          topics::OUTER_COMMAND,
          qos::control_commands(),
          [this](custom_msgs::msg::CommandRequest::SharedPtr msg) {
            if (msg->type == CmdType::HeartBeat) return; // 心跳包
            if (msg->dst != data_cache_->getVehicleId()) {  // 非本机收到命令
              if (msg->type == CmdType::Takeoff || // 起飞
                  msg->type == CmdType::Land || // 降落
                  msg->type == CmdType::Loiter || // 停机
                  msg->type == CmdType::DoTask || // 执行任务
                  msg->type == CmdType::Joystick || // 摇杆控制
                  msg->type == CmdType::DesignAttackObj)// 设计攻击目标
                mission_context_->addExcludedId(msg->dst);
              return;
            }
            custom_msgs::msg::CommandResponse response;
            response.id = data_cache_->getVehicleId();
            response.src = CtrlType::Cmd;
            response.type = msg.get()->type;
            response.status = CmdStatus::Success;

            switch (msg.get()->type) {
              case CmdType::SetVideo: {// 图片视频指令
                custom_msgs::msg::ImageDistribute imgDis;
                imgDis.type = msg->param0;
                imgDis.rcvip = msg->param1;
                imgDis.rcvport = msg->param2;
                imgDis.resx = msg->param3;
                imgDis.resy = msg->param4;
                imgDis.fps = msg->fparam5;
                publish(topics::INFO_IMAGE_DISTRIBUTE, imgDis);
                std::string url;
                if (msg->param0 == 2 && isServiceReady(services::INFO_RTSP_URL)) {
                  auto future = callService<custom_msgs::srv::CommandString>(services::INFO_RTSP_URL, {});
                  if (rclcpp::spin_until_future_complete(node_, future) == rclcpp::FutureReturnCode::SUCCESS) {
                    response.rslt = future.get()->rslt;
                  } else {
                    txtLog().error(THISMODULE "Failed to get RTSP URL");
                  }
                }
                break;
              }
              // TODO:: 以下指令待实现
              case CmdType::Land:
              case CmdType::Return:
              case CmdType::CmdSetHome: {
                if (data_cache_->getVehicleState()->lock == LockState::UNLOCK) {
                  txtLog().warnning(THISMODULE "Vehicle is locked, cannot set home");
                  response.status = CmdStatus::Failed;
                  response.rslt = "Vehicle is locked, cannot set home";
                  break;
                }
                geometry_msgs::msg::Point home_point;
                home_point.x = msg->param0 / 1e7;
                home_point.y = msg->param1 / 1e7;
                home_point.z = msg->param2 / 1e3;
                publish(topics::SET_COORD, home_point);
                data_cache_->updateHomePoint(std::make_shared<geometry_msgs::msg::Point>(home_point));
              }
              case CmdType::SyncTime:
              case CmdType::SetVehi:
              case CmdType::SetImgCh:
              case CmdType::Weapon:
              default:break;
            }
            publish(topics::OUTER_RESPONSE, response
            );
          }
      )
  );
}

void ROSCommunicationManager::setupServiceClients() {
  using namespace ros_interface;

  // 飞行模式服务
  service_clients_[services::SET_FLIGHT_MODE] =
      node_->create_client<custom_msgs::srv::CommandLong>(
          services::SET_FLIGHT_MODE);

  // 锁定/解锁服务
  service_clients_[services::LOCK_UNLOCK] =
      node_->create_client<custom_msgs::srv::CommandBool>(
          services::LOCK_UNLOCK);

  // 编队切换服务
  service_clients_[services::FORMATION_SWITCH] =
      node_->create_client<custom_msgs::srv::CommandInt>(
          services::FORMATION_SWITCH);

  service_clients_[services::INFO_RTSP_URL] =
      node_->create_client<custom_msgs::srv::CommandString>(
          services::INFO_RTSP_URL);

  // 补充的服务
  //    service_clients_[services::ARM_DISARM] =
  //            node_->create_client<custom_msgs::srv::CommandBool>(
  //                    services::ARM_DISARM);
  //
  //    service_clients_[services::EMERGENCY_STOP] =
  //            node_->create_client<std_srvs::srv::Trigger>(
  //                    services::EMERGENCY_STOP);
}

void ROSCommunicationManager::handleMissionJSON(std_msgs::msg::String::SharedPtr msg) {
  try {
    nlohmann::json mission_json = nlohmann::json::parse(msg->data);

    if (!mission_json.contains("stage")) {
      RCLCPP_WARN(node_->get_logger(), "Mission JSON missing 'stage' field");
      return;
    }

    nlohmann::json stage_info = mission_json["stage"];
    if (!stage_info.is_array() || stage_info.empty()) {
      RCLCPP_WARN(node_->get_logger(), "Invalid stage info format");
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
      } else {
        RCLCPP_WARN(node_->get_logger(), "Unknown command: %s", cmd.c_str());
      }
    }

  } catch (const nlohmann::json::exception &e) {
    txtLog().error(THISMODULE "JSON parsing error: %s", e.what());
  } catch (const std::exception &e) {
    txtLog().error(THISMODULE "Error handling mission JSON: %s", e.what());
  }
}

bool ROSCommunicationManager::validateStage(const nlohmann::json &stage) {
  if (!stage.contains("name") || !stage.contains("sn") ||
      !stage.contains("cmd") || !stage.contains("actions")) {
    txtLog().error(THISMODULE
                   "Stage missing required fields: name, sn, cmd, or actions");
    return false;
  }
  return true;
}

void ROSCommunicationManager::processStartCommand(const nlohmann::json &stage) {
  auto actions = stage["actions"];
  if (!actions.is_array() || actions.empty()) {
    return;
  }
  int stage_sn = stage["sn"].get<int>();
  std::string stage_name = stage["name"].get<std::string>();

  // 存储分组信息
  std::unordered_map<int, std::vector<int>> group_info;
  std::set<uint8_t> stage_ids;
  geometry_msgs::msg::Polygon group_offsets;

  // 当前飞机的信息
  int my_id = data_cache_->getVehicleId();
  int my_group_id = -1;
  nlohmann::json my_action;

  // 处理所有动作
  for (const auto &action : actions) {
    if (!validateAction(action)) {
      continue;
    }

    int id = action["id"].get<int>();
    int group_id = action["groupid"].get<int>();

    // 记录执行任务的飞机
    stage_ids.insert(id);

    // 处理分组信息
    if (group_id != -1) {
      group_info[group_id].push_back(id);

      // 检查是否是当前飞机
      if (id == my_id) {
        my_group_id = group_id;
        my_action = action;
      }
    }
  }

  // 如果任务包含当前飞机
  if (my_group_id != -1) {
    processSelfMission(my_action, stage_sn, group_info[my_group_id], actions);
  } else {
    // 更新排除的分组成员
    updateExcludedGroupMembers(stage_ids);
  }
}

bool ROSCommunicationManager::validateAction(const nlohmann::json &action) {
  if (!action.contains("name") || !action.contains("id") ||
      !action.contains("groupid")) {
    txtLog().error(THISMODULE "Action missing required fields: name, id, or groupid");
    return false;
  }
  return true;
}

void ROSCommunicationManager::processSelfMission(
    const nlohmann::json &action,
    int stage_sn,
    const std::vector<int> &group_members,
    const nlohmann::json &all_actions) {

  // 更新任务上下文
  std::string action_name = action["name"].get<std::string>();
  mission_context_->setAction(action_name);
  mission_context_->setStage(stage_sn);
  mission_context_->setGroupId(action["groupid"].get<int>());
  mission_context_->setGroupMembers(group_members);

  // 处理参数
  if (action.contains("params") && action["params"].is_array()) {
    for (const auto &param : action["params"]) {
      if (param.contains("name") && param.contains("value")) {
        std::string param_name = param["name"].get<std::string>();
        mission_context_->setParameter(param_name, param["value"]);
      }
    }
  }

  // 处理触发器
  if (action.contains("triggers") && action["triggers"].is_array()) {
    for (const auto &trigger : action["triggers"]) {
      if (trigger.contains("name") && trigger.contains("value")) {
        std::string trigger_name = trigger["name"].get<std::string>();
        mission_context_->setTrigger(trigger_name, trigger["value"]);
      }
    }
  }

  // 处理编队偏移
  processFormationOffsets(all_actions, action["groupid"].get<int>());

  // 发布初始任务状态
  publishTaskStatus(stage_sn, StatusStage::StsNoStart);

  // 请求加载对应的行为树
  std::string tree_name = action_name + "_start";
  requestTreeLoad(tree_name);

  // 清理排除ID
  mission_context_->clearExcludedIds();
}

void ROSCommunicationManager::processFormationOffsets(
    const nlohmann::json &actions,
    int group_id) {

  geometry_msgs::msg::Polygon offsets;

  for (const auto &action : actions) {
    if (action["groupid"].get<int>() != group_id) {
      continue;
    }

    if (!action.contains("params") || !action["params"].is_array()) {
      continue;
    }

    // 查找编队偏移参数
    for (const auto &param : action["params"]) {
      if (param["name"].get<std::string>() == "formOffset") {
        auto value = param["value"];
        if (value.contains("diff_x_lat") &&
            value.contains("diff_y_lon") &&
            value.contains("diff_z_alt")) {

          geometry_msgs::msg::Point32 offset;
          offset.x = value["diff_x_lat"].get<float>();
          offset.y = value["diff_y_lon"].get<float>();
          offset.z = value["diff_z_alt"].get<float>();
          offsets.points.push_back(offset);
        }
      }
    }
  }

  // 发布编队偏移
  if (!offsets.points.empty()) {
    publish(ros_interface::topics::INFO_GROUP_OFFSET, offsets);
  }
}

void ROSCommunicationManager::processStopCommand(const nlohmann::json &stage) {
  int stage_sn = stage["sn"].get<int>();

  // 检查是否是当前执行的任务
  if (mission_context_->getStage() == stage_sn) {
    // 停止当前行为树
    requestTreeStop();

    // 发布任务停止状态
    publishTaskStatus(stage_sn, StatusStage::StsFailed);

    // 清理任务上下文
    mission_context_->clear();
  }
}

void ROSCommunicationManager::processInsertCommand(const nlohmann::json &stage) {
  // 插入命令通常用于动态添加任务
  // 这里可以实现任务队列管理
  txtLog().info(THISMODULE "Insert command received for stage: %s",
                stage["name"].get<std::string>().c_str());

  // TODO: 实现任务队列管理
}

void ROSCommunicationManager::updateExcludedGroupMembers(const std::set<uint8_t> &executing_ids) {

  // 获取当前分组成员
  auto current_members = mission_context_->getGroupMembers();
  std::vector<int> updated_members;

  // 过滤掉正在执行其他任务的成员
  for (int member_id : current_members) {
    if (executing_ids.find(member_id) == executing_ids.end()) {
      updated_members.push_back(member_id);
    }
  }

  // 如果有成员被排除
  if (updated_members.size() != current_members.size()) {
    // 更新分组成员
    std_msgs::msg::UInt8MultiArray group_ids_msg;
    group_ids_msg.data.assign(updated_members.begin(), updated_members.end());
    publish(ros_interface::topics::INFO_GROUP_IDS, group_ids_msg);

    // 更新任务上下文中的排除ID
    mission_context_->setExcludedIds(executing_ids);
    publish(ros_interface::topics::INFO_GROUP_IDS, mission_context_->getGroupMembers());
  }
}

void ROSCommunicationManager::publishTaskStatus(int stage_sn, StatusStage status) {
  custom_msgs::msg::StatusTask status_msg;
  status_msg.stage = stage_sn;
  status_msg.id = data_cache_->getVehicleId();
  status_msg.status = static_cast<int>(status);

  publish(ros_interface::topics::OUTER_STATUS_TASK, status_msg);
}
