#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <memory>

// 消息类型
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <custom_msgs/msg/simple_vehicle.hpp>
#include <custom_msgs/msg/offboard_ctrl.hpp>
#include <custom_msgs/msg/object_computation.hpp>
#include <custom_msgs/msg/task_stage.hpp>
#include <custom_msgs/msg/status_task.hpp>

// 服务类型
#include <custom_msgs/srv/command_bool.hpp>
#include <custom_msgs/srv/command_int.hpp>
#include <custom_msgs/srv/command_long.hpp>
#include <custom_msgs/srv/command_string.hpp>

#include "behavior_test_simulation/vehicle_simulator.hpp"

using namespace std::chrono_literals;

// 定义仿真对象结构体
struct SimulatedObject {
  uint32_t id;
  float x, y, z;
  float vx, vy, vz;
  float confidence;
  std::string type;

  SimulatedObject(uint32_t id, float x, float y, float z,
                  float vx = 0.0f, float vy = 0.0f, float vz = 0.0f,
                  float confidence = 0.8f, std::string type = "target")
      : id(id), x(x), y(y), z(z), vx(vx), vy(vy), vz(vz),
        confidence(confidence), type(type) {}
};

class SimulationNode : public rclcpp::Node {
 public:
  SimulationNode() : Node("simulation_node"), vehicle_sim_(1) {
    setupPublishers();
    setupSubscribers();
    setupServices();
    setupTimers();

    RCLCPP_INFO(get_logger(), "Simulation Node initialized");
  }

 private:
  // 仿真器
  VehicleSimulator vehicle_sim_;

  // 发布器
  rclcpp::Publisher<custom_msgs::msg::SimpleVehicle>::SharedPtr vehicle_state_pub_;
  rclcpp::Publisher<custom_msgs::msg::ObjectComputation>::SharedPtr object_detection_pub_;

  // 订阅器
  rclcpp::Subscription<custom_msgs::msg::OffboardCtrl>::SharedPtr offboard_ctrl_sub_;
  rclcpp::Subscription<custom_msgs::msg::StatusTask>::SharedPtr status_task_sub_;

  // 服务
  rclcpp::Service<custom_msgs::srv::CommandBool>::SharedPtr lock_service_;
  rclcpp::Service<custom_msgs::srv::CommandLong>::SharedPtr flight_mode_service_;
  rclcpp::Service<custom_msgs::srv::CommandInt>::SharedPtr formation_service_;
  rclcpp::Service<custom_msgs::srv::CommandString>::SharedPtr rtsp_service_;

  // 定时器
  rclcpp::TimerBase::SharedPtr vehicle_update_timer_;
  rclcpp::TimerBase::SharedPtr object_detection_timer_;

  // 仿真参数
  bool simulate_objects_ = true;
  std::vector<SimulatedObject> simulated_objects_;  // 修正：使用自定义结构体

  void setupPublishers() {
    // 飞机状态发布
    vehicle_state_pub_ = create_publisher<custom_msgs::msg::SimpleVehicle>(
        "inner/information/simple_vehicle", 10);

    // 目标检测发布
    object_detection_pub_ = create_publisher<custom_msgs::msg::ObjectComputation>(
        "inner/information/object_computation", 10);
  }

  void setupSubscribers() {
    // 订阅外部控制指令
    offboard_ctrl_sub_ = create_subscription<custom_msgs::msg::OffboardCtrl>(
        "inner/control/offboard", 10,
        [this](custom_msgs::msg::OffboardCtrl::SharedPtr msg) {
          handleOffboardControl(msg);
        });

    // 订阅任务状态
    status_task_sub_ = create_subscription<custom_msgs::msg::StatusTask>(
        "outer/information/status_task", 10,
        [this](custom_msgs::msg::StatusTask::SharedPtr msg) {
          handleStatusTask(msg);
        });
  }

  void setupServices() {
    // 锁定/解锁服务
    lock_service_ = create_service<custom_msgs::srv::CommandBool>(
        "inner/control/lock_unlock",
        [this](const std::shared_ptr<custom_msgs::srv::CommandBool::Request> request,
               std::shared_ptr<custom_msgs::srv::CommandBool::Response> response) {
          vehicle_sim_.setLockState(request->value);
          response->success = true;
          RCLCPP_INFO(get_logger(), "Vehicle %s", request->value ? "unlocked" : "locked");
        });

    // 飞行模式设置服务
    flight_mode_service_ = create_service<custom_msgs::srv::CommandLong>(
        "inner/control/set_flymode",
        [this](const std::shared_ptr<custom_msgs::srv::CommandLong::Request> request,
               std::shared_ptr<custom_msgs::srv::CommandLong::Response> response) {
          vehicle_sim_.setFlightMode(static_cast<uint8_t>(request->command));

          // 模拟起飞逻辑
          if (request->command == 4) { // TakeOff mode
            double takeoff_alt = request->param7;
            if (takeoff_alt != 0) {
              auto pos = vehicle_sim_.getCurrentPosition();
              vehicle_sim_.setPosition(pos.x, pos.y, -std::abs(takeoff_alt), pos.yaw);
            }
          }

          response->success = true;
          RCLCPP_INFO(get_logger(), "Flight mode set to: %d", request->command);
        });

    // 编队控制服务
    formation_service_ = create_service<custom_msgs::srv::CommandInt>(
        "inner/control/form_switch",
        [this](const std::shared_ptr<custom_msgs::srv::CommandInt::Request> request,
               std::shared_ptr<custom_msgs::srv::CommandInt::Response> response) {
          response->success = true;
          RCLCPP_INFO(get_logger(), "Formation command: frame=%d, command=%d",
                      request->frame, request->command);
        });

    // RTSP URL获取服务
    rtsp_service_ = create_service<custom_msgs::srv::CommandString>(
        "inner/get/rtsp_url",
        [this](const std::shared_ptr<custom_msgs::srv::CommandString::Request> request,
               std::shared_ptr<custom_msgs::srv::CommandString::Response> response) {
          response->rslt = "rtsp://127.0.0.1:8554/test_stream";
          RCLCPP_INFO(get_logger(), "RTSP URL requested");
        });
  }

  void setupTimers() {
    // 飞机状态更新定时器 (50Hz)
    vehicle_update_timer_ = create_wall_timer(
        20ms, [this]() { updateVehicleState(); });

    // 目标检测定时器 (10Hz)
    object_detection_timer_ = create_wall_timer(
        100ms, [this]() { publishObjectDetection(); });

    // 初始化仿真目标
    initializeSimulatedObjects();
  }

  void updateVehicleState() {
    // 更新仿真器状态
    vehicle_sim_.update(0.02); // 20ms时间步长

    // 发布飞机状态
    auto state = vehicle_sim_.getVehicleState();
    vehicle_state_pub_->publish(state);
  }

  void publishObjectDetection() {
    if (!simulate_objects_ || simulated_objects_.empty()) return;

    custom_msgs::msg::ObjectComputation obj_computation_msg;

    // 添加一些噪声到目标位置，模拟真实检测的不确定性
    static std::mt19937 rng(std::random_device{}());
    std::uniform_real_distribution<float> noise(-1.0f, 1.0f);

    // 清空之前的对象列表
    obj_computation_msg.objs.clear();

    for (const auto& sim_obj : simulated_objects_) {
      // 创建检测到的对象消息
      custom_msgs::msg::ObjectLocation detected_obj;

      // 基本信息
      detected_obj.id = sim_obj.id;

      // 位置信息（添加噪声模拟检测误差）
      detected_obj.x = sim_obj.x + noise(rng) * 0.5f;
      detected_obj.y = sim_obj.y + noise(rng) * 0.5f;
      detected_obj.z = sim_obj.z + noise(rng) * 0.2f;

      // 速度信息（添加噪声）
      detected_obj.vx = sim_obj.vx + noise(rng) * 0.1f;
      detected_obj.vy = sim_obj.vy + noise(rng) * 0.1f;
      detected_obj.vz = sim_obj.vz + noise(rng) * 0.05f;

      // 添加到检测结果中
      obj_computation_msg.objs.push_back(detected_obj);
    }

    // 更新仿真对象位置（简单的运动模拟）
    updateSimulatedObjectPositions(0.1); // 100ms时间步长

    // 发布目标检测结果
    object_detection_pub_->publish(obj_computation_msg);

    RCLCPP_DEBUG(get_logger(), "Published %zu detected objects",
                 obj_computation_msg.objs.size());
  }

  void initializeSimulatedObjects() {
    // 创建一些仿真目标
    simulated_objects_.clear();

    // 目标1：移动车辆
    simulated_objects_.emplace_back(
        101,          // id
        100.0f,       // x
        50.0f,        // y
        -30.0f,       // z
        2.0f,         // vx
        1.0f,         // vy
        0.0f,         // vz
        0.85f,        // confidence
        "vehicle"     // type
    );

    // 目标2：移动人员
    simulated_objects_.emplace_back(
        102,          // id
        -80.0f,       // x
        120.0f,       // y
        -25.0f,       // z
        -1.5f,        // vx
        -0.5f,        // vy
        0.2f,         // vz
        0.92f,        // confidence
        "person"      // type
    );

    // 目标3：静态建筑
    simulated_objects_.emplace_back(
        103,          // id
        200.0f,       // x
        200.0f,       // y
        -15.0f,       // z
        0.0f,         // vx
        0.0f,         // vy
        0.0f,         // vz
        0.95f,        // confidence
        "building"    // type
    );

    RCLCPP_INFO(get_logger(), "Initialized %zu simulated objects",
                simulated_objects_.size());
  }

  void updateSimulatedObjectPositions(double dt) {
    // 更新仿真对象的位置
    for (auto& obj : simulated_objects_) {
      // 只更新移动目标的位置
      if (obj.type != "building" && obj.type != "static") {
        obj.x += obj.vx * dt;
        obj.y += obj.vy * dt;
        obj.z += obj.vz * dt;

        // 简单的边界处理：如果超出范围就反向
        if (std::abs(obj.x) > 300.0f) obj.vx *= -1;
        if (std::abs(obj.y) > 300.0f) obj.vy *= -1;
        if (obj.z > -10.0f || obj.z < -60.0f) obj.vz *= -1;
      }
    }
  }

  void handleOffboardControl(custom_msgs::msg::OffboardCtrl::SharedPtr msg) {
    // 将控制指令传递给仿真器
    vehicle_sim_.setTarget(*msg);

    RCLCPP_DEBUG(get_logger(), "Received offboard control: x=%.2f, y=%.2f, z=%.2f, yaw=%.2f",
                 msg->x, msg->y, msg->z, msg->yaw);
  }

  void handleStatusTask(custom_msgs::msg::StatusTask::SharedPtr msg) {
    RCLCPP_INFO(get_logger(), "Task status: stage=%d, id=%d, status=%d",
                msg->stage, msg->id, msg->status);
  }

 public:
  // 用于测试的公共接口
  void setSimulateObjects(bool enable) {
    simulate_objects_ = enable;
    RCLCPP_INFO(get_logger(), "Object simulation %s", enable ? "enabled" : "disabled");
  }

  void addSimulatedObject(uint32_t id, float x, float y, float z,
                          float vx = 0.0f, float vy = 0.0f, float vz = 0.0f,
                          float confidence = 0.8f, const std::string& type = "custom") {
    simulated_objects_.emplace_back(id, x, y, z, vx, vy, vz, confidence, type);
    RCLCPP_INFO(get_logger(), "Added simulated object: id=%u, pos=(%.1f,%.1f,%.1f)",
                id, x, y, z);
  }

  void clearSimulatedObjects() {
    simulated_objects_.clear();
    RCLCPP_INFO(get_logger(), "Cleared all simulated objects");
  }

  void setVehiclePosition(double x, double y, double z, double yaw = 0.0) {
    vehicle_sim_.setPosition(x, y, z, yaw);
    RCLCPP_INFO(get_logger(), "Set vehicle position: (%.2f, %.2f, %.2f, %.2f)",
                x, y, z, yaw);
  }

  size_t getSimulatedObjectCount() const {
    return simulated_objects_.size();
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<SimulationNode>();

  RCLCPP_INFO(node->get_logger(), "Starting behavior test simulation...");

  try {
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node->get_logger(), "Exception: %s", e.what());
  }

  rclcpp::shutdown();
  return 0;
}