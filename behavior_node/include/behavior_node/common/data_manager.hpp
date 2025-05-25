#pragma once

#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <log/Logger.hpp>

#include <mavros_msgs/msg/state.hpp>
#include <custom_msgs/msg/simple_vehicle.hpp>
#include <custom_msgs/msg/dis_target.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <custom_msgs/msg/command_request.hpp>
#include <custom_msgs/msg/task_stage.hpp>
#include <custom_msgs/msg/object_attack_designate.hpp>
#include <custom_msgs/msg/object_computation.hpp>
#include <custom_msgs/msg/detail/status_task__struct.hpp>
#include <behaviortree_cpp/bt_factory.h>

#include "nlohmann/json.hpp"
#include "behavior_node/common/base_enum.hpp"
#include "behavior_node/action/flight_mode_control.hpp"
#include "behavior_node/action/off_board_control.hpp"
#include "behavior_node/action/navigation_control.hpp"
#include "behavior_node/action/lock_control.hpp"
#include "behavior_node/action/set_line_parameters.hpp"
#include "behavior_node/action/set_destination_point.hpp"
#include "behavior_node/condition/check_quit_search.hpp"
#include "behavior_node/condition/check_arrive_destination.hpp"

class DataManager {
 public:
  static DataManager &getInstance() {
    if (!instance_) {
      instance_ = new DataManager();
    }
    return *instance_;
  }

  void initialize(const rclcpp::Node::SharedPtr &node) {
    //创建各种发布
    status_task_pub_ = node->create_publisher<custom_msgs::msg::StatusTask>("outer/information/status_task", 1);
    group_member_ids_pub_ = node->create_publisher<std_msgs::msg::UInt8MultiArray>("inner/information/group_ids", 1);
    group_offsets_pub_ = node->create_publisher<geometry_msgs::msg::Polygon>("inner/information/group_offset", 1);
    way_pts_pub_ = node->create_publisher<geometry_msgs::msg::Polygon>("inner/set/navline", 1);

    // 创建各种订阅
    // 订阅简单飞控信息
    add_subscription(node, "inner/information/simple_vehicle", simple_vehicle_);
    add_subscription(node, "inner/information/object_computation", object_computation_);
    add_subscription(node, "inner/information/way_point", current_navigation_info_);
    add_subscription(node, "inner/control/joystick", joystick_);
    add_subscription(node, "inner/information/gpio_status", gpio_status_);

    add_subscription<std_msgs::msg::String>(
        node, "outer/set/stage_info_json",
        [this](std::shared_ptr<std_msgs::msg::String> msg) {
          // TODO:: 待补全 参考 home\h\ros2_ws\src\behavior_lib\src\control\TaskDecision.cpp208行
          nlohmann::json states = nlohmann::json::parse(msg->data);
          if (states.contains("stage")) {
            nlohmann::json stage_info = states["stage"];
            if (stage_info.is_array() && !stage_info.empty()) {
              for (auto stage : stage_info) {
                if (!(stage.contains("name") && stage.contains("sn") && stage.contains("cmd")
                    && stage.contains("actions"))) {
                  txtLog().error(THISMODULE "stage err, without name、 sn、cmd、actions");
                  return;
                }
                if (auto cmd = stage["cmd"].get<std::string>();cmd != "del" || cmd != "set" || cmd != "ins") {
                  return;
                } else {
                  auto actions = stage["actions"];
                  if (actions.is_array() && !actions.empty()) {
                    std::unordered_map<int, std::vector<int>> group_info;
                    std::set<uint8_t> stage_ids;
                    offsets_.points.clear();
                    int group_belong_to = -1;
                    for (auto action : actions) {
                      if (action.contains("name") && action.contains("id") && action.contains("groupid")) {
                        int id = action["id"].get<int>();
                        if (cmd == "start") {
                          stage_ids.insert(id); // 只添加开始任务的飞机
                        }

                        // 存在groupid
                        if (auto group_id = action["groupid"].get<int>(); group_id != -1) {
                          // 如果groupid不存在，则创建，
                          if (group_info.count(group_id) == 0) {
                            group_info[group_id] = std::vector<int>();
                          }
                          // group中不存在id，则创建
                          if (std::find(group_info[group_id].begin(), group_info[group_id].end(), id)
                              == group_info[group_id].end()) {
                            // 记录组成员的id
                            group_info[group_id].push_back(id);
                          }

                          if (id == getId()) {
                            group_belong_to = group_id;
                            // 设置当前飞机所属编组id
                            group_id_ = group_id;
                            // 设置当前任务名
                            action_name_ = action["name"].get<std::string>();
                            // 存储json参数
                            for (auto param : action["params"]) {
                              if (param.contains("name") && param.contains("value")) {
                                json_params_[param["name"].get<std::string>()] = param["value"];
                              }
                            }
                            // 存储json的触发器
                            for (auto trigger : action["triggers"]) {
                              if (trigger.contains("name") && trigger.contains("value")) {
                                json_triggers_[trigger["name"].get<std::string>()] = trigger["value"];
                              }
                            }

                            // 清理排除id
                            clearExtIds();

                            std::string treeName = action_name_ + cmd;
                            // 加载行为树
                            loadBehaviorTree(treeName);
                            // 设置标志位，由json控制为真
                            control_with_json_ = true;
                            // 初始化任务状态
                            custom_msgs::msg::StatusTask status_task;
                            status_task.stage = actions["sn"].get<int>();
                            status_task.id = id;
                            status_task.status = StatusStage::StsNoStart;
                            status_task_pub_->publish(status_task);
                            current_navigation_info_.id = -1;
                            current_navigation_info_.dis = 0;
                            pre_nav_id_ = -1;
                          }
                        }
                      } else {
                        txtLog().error(THISMODULE "action err, without name、id、groupid");
                      }
                    }
                    // 如果当前任务包含本飞机
                    if (group_belong_to != -1) {
                      // 设置当前组成员
                      group_member_ids_.assign(group_info[group_belong_to].begin(), group_info[group_belong_to].end());
                      // 清除队形偏移量
                      offsets_.points.clear();
                      // 遍历action
                      for (auto action : actions) {
                        // 当前编组
                        if (auto group_id = action["groupid"].get<int>(); group_id == group_belong_to) {
                          // 有参数的action
                          if (auto params = action["params"]; params.is_array() && !params.empty()) {
                            for (auto param : params) {
                              // 编队偏移参数
                              if (auto name = param["name"].get<std::string>(); name == "formOffset") {
                                if (auto value = param["value"]; value.contains("diff_x_lat")
                                    && value.contains("diff_y_lon") && value.contains("diff_z_alt")) {
                                  geometry_msgs::msg::Point32 offset;
                                  offset.x = value["diff_x_lat"].get<double>();
                                  offset.y = value["diff_y_lon"].get<double>();
                                  offset.z = value["diff_z_alt"].get<double>();
                                  offsets_.points.push_back(offset);
                                }
                              }
                            }
                          }
                        }
                      }

                    } else if (!stage_ids.empty()) { //不为本机执行任务，需要把开始执行任务的飞机进行分组排除
                      std::vector<int> temp_ids{group_member_ids_.begin(), group_member_ids_.end()};
                      ext_ids_.insert(stage_ids.begin(), stage_ids.end());
                      for (auto id : ext_ids_) {
                        if (auto it = std::find(temp_ids.begin(), temp_ids.end(), id);it != temp_ids.end()) {
                          temp_ids.erase(it);
                        }
                      }
                      // 如果确实有被排除的id，则重新设置分组
                      if (ext_ids_.size() != temp_ids.size()) {
                        std_msgs::msg::UInt8MultiArray group_ids;
                        group_ids.data.assign(temp_ids.begin(), temp_ids.end());
                        group_member_ids_pub_->publish(group_ids);

                        geometry_msgs::msg::Polygon offsets;

                        // 移除offsets中，下标为ext_ids中的值在group_member_ids_的下标的元素
                        for (auto id : ext_ids_) {
                          for (auto i = 0; i < group_member_ids_.size(); i++) {
                            if (group_member_ids_[i] == id) {
                              offsets.points.erase(offsets.points.begin() + i);
                              break;
                            }
                          }
                        }
                        if (!offsets.points.empty()) {
                          group_offsets_pub_->publish(offsets);
                        }
//                        // 目前判断条件恒为假，暂不实现
//                        if(isSwitchSetLine())
//                          setWayPts(m_s_wayPts);
//                        if(isSwitchSetLoops())
//                          setLoops(m_s_loops);

                      }
                    }
                  }
                }
              }
            }
          }
        }
    );

    add_subscription<custom_msgs::msg::CommandRequest>(
        node,
        "outer/command/request",
        [this](
            std::shared_ptr<custom_msgs::msg::CommandRequest> msg
        ) {
          // TODO:: 待补全 参考 home\h\ros2_ws\src\behavior_lib\src\control\TaskDecision.cpp212行
        });

    add_subscription<custom_msgs::msg::TaskStage>(
        node,
        "outer/set/task_stage",
        [this](
            std::shared_ptr<custom_msgs::msg::TaskStage> msg
        ) {
          // TODO:: 待补全 参考 home\h\ros2_ws\src\behavior_lib\src\control\TaskDecision.cpp214行
        });

    add_subscription<custom_msgs::msg::ObjectAttackDesignate>(
        node,
        "outer/set/attack_object_designate",
        [this](
            std::shared_ptr<custom_msgs::msg::ObjectAttackDesignate> msg
        ) {
          // TODO:: 待补全 参考 home\h\ros2_ws\src\behavior_lib\src\control\TaskDecision.cpp216行
        });

    add_subscription<custom_msgs::msg::SimpleVehicle>(
        node,
        "outer/information/simple_vehicle",
        [this](
            std::shared_ptr<custom_msgs::msg::SimpleVehicle> msg
        ) {
          this->vehicle_map_[msg->id] = *msg;
        });
  }

// 模板函数，添加订阅，模板类型为消息类型
  template<typename msgType>
  void add_subscription(const rclcpp::Node::SharedPtr &node, const std::string &topic, msgType &target) {
    subscriptions_.push_back(
        node->create_subscription<msgType>(
            topic, rclcpp::SensorDataQoS(),
            [&target, this](std::shared_ptr<msgType> msg) {
              std::lock_guard<std::mutex> lock(mutex_);
              target = *msg;
            }
        )
    );
  }

  template<typename msgType>
  void add_subscription(const rclcpp::Node::SharedPtr &node,
                        const std::string &topic,
                        std::function<void(const std::shared_ptr<msgType> &)> callback) {
    subscriptions_.push_back(
        node->create_subscription<msgType>(
            topic, rclcpp::SensorDataQoS(),
            [callback, this](std::shared_ptr<msgType> msg) {
              std::lock_guard<std::mutex> lock(mutex_);
              callback(msg);
            }
        )
    );
  }

// BT相关函数
// 注册BT节点
  void registerBTNodes(const rclcpp::Node::SharedPtr &node) {

    // TODO:: 此处存在循环引用，待解决
    // 注册动作节点
    factory_.registerNodeType<FlightModeControl>("FlightModeControl", node);

    factory_.registerNodeType<OffBoardControl>("OffBoardControl", node);

    factory_.registerNodeType<NavigationControl>("NavigationControl", node);

    factory_.registerNodeType<LockControl>("LockControl", node);

    factory_.registerNodeType<SetLineParameters>("SetLineParameters", node);

    factory_.registerNodeType<SetDestinationPoint>("SetDestinationPoint");

    // 注册条件节点
    factory_.registerNodeType<CheckArriveDestination>("CheckArriveDestination");

    factory_.registerNodeType<CheckQuitSearch>("CheckQuitSearch");

  }

// 初始化BT树对象，加载xml文件
  void initialize_tree_(const rclcpp::Node::SharedPtr &node, const std::string &tree_dir) {
    // 注册BT节点
    registerBTNodes(node);

    // 加载行为树文件映射
    std::string bt_dir = tree_dir + "/tree";
    if (!std::filesystem::exists(bt_dir)) {
      txtLog().error(THISMODULE "行为树目录 %s 不存在", bt_dir.c_str());
      return;
    }

    for (const auto &entry : std::filesystem::directory_iterator(bt_dir)) {
      if (entry.is_regular_file() && entry.path().extension() == ".xml") {
        factory_.registerBehaviorTreeFromFile(entry.path().string());
        txtLog().info(THISMODULE "加载行为树文件: %s", entry.path().c_str());
      }
    }
  }

//
  void loadBehaviorTree(const std::string &tree_name) {
    try {
      // 加载行为树
      tree_ = std::make_unique<BT::Tree>(factory_.createTree(tree_name));

      current_tree_name_ = tree_name;
      tree_running_ = true;

      txtLog().info(THISMODULE "已加载行为树: %s", tree_name.c_str());
    } catch (const std::exception &e) {
      txtLog().info(THISMODULE "加载行为树失败: %s", e.what());
    }
  }

  void switchBehaviorTree(const std::string &tree_name) {
    if (tree_running_) {
      // 如果当前行为树正在运行，先停止
      if (tree_) {
        tree_->haltTree();
      }
      tree_running_ = false;
    }

    // 加载新的行为树
    loadBehaviorTree(tree_name);
  }

  void executeCycle() {
    if (tree_ && tree_running_) {
      BT::NodeStatus status = tree_->tickOnce();

      if (status == BT::NodeStatus::SUCCESS) {
        txtLog().info(THISMODULE "行为树 %s 执行成功", current_tree_name_.c_str());
        tree_running_ = false;
      } else if (status == BT::NodeStatus::FAILURE) {
        txtLog().info(THISMODULE "行为树 %s 执行失败", current_tree_name_.c_str());
        tree_running_ = false;
      }
    }
  }

  u_int8_t getId() const {
    return simple_vehicle_.id;
  }

  bool setHome(geometry_msgs::msg::Point home) {
    // 同步锁
    std::lock_guard<std::mutex> lock(mutex_);
    this->home_ = home;
    return true;
  }

  bool setGroupId(int i) {
    std::lock_guard<std::mutex> lock(mutex_);
    group_id_ = i;
  }

  bool setGroupMemberId(std::vector<int> group_member_id) {
    std::lock_guard<std::mutex> lock(mutex_);
    group_member_ids_.assign(group_member_id.begin(), group_member_id.end());
  }

  geometry_msgs::msg::Point getHome() {
    return home_;
  }

  bool setJsonParams(const std::string &action_name, nlohmann::json json_params) {
    // 同步锁
    std::lock_guard<std::mutex> lock(mutex_);
    this->json_params_[action_name] = std::move(json_params);
    return true;
  }

  nlohmann::json getJsonParams(const std::string &action_name) {
    if (json_params_.count(action_name) == 0) {
      return {};
    }
    return json_params_[action_name];
  }

  bool setJsonTriggers(const std::string &action_name, nlohmann::json json_triggers) {
    // 同步锁
    std::lock_guard<std::mutex> lock(mutex_);
    this->json_triggers_[action_name] = std::move(json_triggers);
    return true;
  }

  nlohmann::json getJsonTriggers(const std::string &action_name) {
    if (json_triggers_.count(action_name) == 0) {
      return {};
    }
    return json_triggers_[action_name];
  }

  bool setActionName(std::string action_name) {
    // 同步锁
    std::lock_guard<std::mutex> lock(mutex_);
    this->action_name_ = std::move(action_name);
    return true;
  }

  std::string getActionName() {
    return action_name_;
  }

  bool setFlightMode(FlightMode flight_mode) {
    // 同步锁
    std::lock_guard<std::mutex> lock(mutex_);
    this->flight_mode_ = flight_mode;
    return true;
  }

  FlightMode getFlightMode() {
    return flight_mode_;
  }

  bool setSimpleVehicle(custom_msgs::msg::SimpleVehicle simple_vehicle) {
    // 同步锁
    std::lock_guard<std::mutex> lock(mutex_);
    this->simple_vehicle_ = simple_vehicle;
    return true;
  }

  custom_msgs::msg::SimpleVehicle getSimpleVehicle() {
    return simple_vehicle_;
  }

  bool setVehicleType(VehicleType vehicle_type) {
    // 同步锁
    std::lock_guard<std::mutex> lock(mutex_);
    this->vehicle_type_ = vehicle_type;
    return true;
  }

  VehicleType getVehicleType() {
    return vehicle_type_;
  }

  bool setTargetId(uint8_t target_id) {
    // 同步锁
    std::lock_guard<std::mutex> lock(mutex_);
    this->target_id_ = target_id;
    return true;
  }

  uint8_t getTargetId() {
    return target_id_;
  }

  custom_msgs::msg::ObjectComputation getObjectComputation() {
    return object_computation_;
  }

  bool clearExtIds() {
    // 同步锁
    std::lock_guard<std::mutex> lock(mutex_);
    ext_ids_.clear();
    return true;
  }

 private:
  DataManager() {};
  ~
  DataManager() = default;
// 同步锁
  std::mutex mutex_{};

  FlightMode flight_mode_{FlightMode::Unknown};// 飞行模式

  geometry_msgs::msg::Point home_{rosidl_runtime_cpp::MessageInitialization::ZERO};// 设置的home点

  std::map<std::string, nlohmann::json> json_params_{};// json任务中的参数信息<参数名，参数值>

  std::map<std::string, nlohmann::json> json_triggers_{};// json任务中的触发信息

  std::string action_name_;// 当前执行的行为名称

  custom_msgs::msg::SimpleVehicle simple_vehicle_{};// 精简飞控消息

  std::unordered_map<u_int8_t, custom_msgs::msg::SimpleVehicle> vehicle_map_{};// 所有飞控的精简消息

  VehicleType vehicle_type_{VehicleType::FixWing};// 飞机类型

  custom_msgs::msg::ObjectComputation object_computation_{};// 目标计算消息

  sensor_msgs::msg::Joy joystick_{};// 遥控器消息

  std_msgs::msg::UInt8 gpio_status_{};// 武器状态

  uint8_t target_id_{0};// 目标id

  uint8_t group_id_{0};// 编组id

  geometry_msgs::msg::Polygon offsets_;//分组偏移

  custom_msgs::msg::DisTarget current_navigation_info_{};//当前目标航点信息

  geometry_msgs::msg::Polygon way_pts_;//航线点

  int pre_nav_id_{-1};//上一个航点id

  std_msgs::msg::UInt8MultiArray::_data_type group_member_ids_{0};// 编组成员id

  std::set<uint8_t> ext_ids_;//排除分组id

  bool control_with_json_;

  custom_msgs::msg::StatusTask status_task_{};// 任务状态消息

  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_{};// 订阅列表

  rclcpp::Publisher<custom_msgs::msg::StatusTask>::SharedPtr status_task_pub_{};// 任务状态发布

  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr group_member_ids_pub_{};// 编组成员id发布

  rclcpp::Publisher<geometry_msgs::msg::Polygon>::SharedPtr group_offsets_pub_{};// 编组分组偏移发布

  rclcpp::Publisher<geometry_msgs::msg::Polygon>::SharedPtr way_pts_pub_;// 航点消息发布对象

  static DataManager *instance_;

// BT相关成员
  BT::BehaviorTreeFactory factory_;
  std::unique_ptr<BT::Tree> tree_{};

// 参数
  std::string config_dir_{"tree"};
  std::string current_tree_name_{"takeoff"};

// 当前行为树状态
  bool tree_running_{false};

};
