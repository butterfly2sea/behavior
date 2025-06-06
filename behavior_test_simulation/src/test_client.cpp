#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <custom_msgs/msg/command_request.hpp>
#include <custom_msgs/msg/task_stage.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nlohmann/json.hpp>
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

class TestClient : public rclcpp::Node {
 public:
  TestClient() : Node("test_client") {
    setupPublishers();

    RCLCPP_INFO(get_logger(), "Test Client initialized");
  }

 private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mission_json_pub_;
  rclcpp::Publisher<custom_msgs::msg::CommandRequest>::SharedPtr command_pub_;
  rclcpp::Publisher<custom_msgs::msg::TaskStage>::SharedPtr task_stage_pub_;

  void setupPublishers() {
    mission_json_pub_ = create_publisher<std_msgs::msg::String>(
        "outer/set/stage_info_json", 10);

    command_pub_ = create_publisher<custom_msgs::msg::CommandRequest>(
        "outer/command/request", 10);

    task_stage_pub_ = create_publisher<custom_msgs::msg::TaskStage>(
        "outer/set/task_stage", 10);
  }

 public:
  void sendSetHomeCommand(double lat = 39.9042, double lon = 116.4074, double alt = 50.0) {
    custom_msgs::msg::CommandRequest cmd;
    cmd.type = 6; // CmdSetHome
    cmd.dst = 1;  // vehicle ID
    cmd.param0 = static_cast<int32_t>(lat * 1e7);
    cmd.param1 = static_cast<int32_t>(lon * 1e7);
    cmd.param2 = static_cast<int32_t>(alt * 1e3);

    command_pub_->publish(cmd);
    RCLCPP_INFO(get_logger(), "Sent SetHome command: lat=%.6f, lon=%.6f, alt=%.1f", lat, lon, alt);
  }

  void sendTakeoffMission() {
    nlohmann::json mission_json;
    mission_json["stage"] = nlohmann::json::array();

    nlohmann::json takeoff_stage;
    takeoff_stage["name"] = "TakeOff";
    takeoff_stage["sn"] = 1;
    takeoff_stage["cmd"] = "set";

    takeoff_stage["actions"] = nlohmann::json::array();
    nlohmann::json action;
    action["name"] = "TakeOff";
    action["id"] = 1;
    action["groupid"] = 1;

    // 添加参数
    action["params"] = nlohmann::json::array();
    nlohmann::json alt_param;
    alt_param["name"] = "alt";
    alt_param["value"] = 30.0;
    action["params"].push_back(alt_param);

    takeoff_stage["actions"].push_back(action);
    mission_json["stage"].push_back(takeoff_stage);

    std_msgs::msg::String msg;
    msg.data = mission_json.dump();
    mission_json_pub_->publish(msg);

    RCLCPP_INFO(get_logger(), "Sent TakeOff mission JSON");
  }

  void sendGotoDstMission(double x = 50.0, double y = 30.0, double z = -25.0) {
    nlohmann::json mission_json;
    mission_json["stage"] = nlohmann::json::array();

    nlohmann::json goto_stage;
    goto_stage["name"] = "GotoDst";
    goto_stage["sn"] = 2;
    goto_stage["cmd"] = "set";

    goto_stage["actions"] = nlohmann::json::array();
    nlohmann::json action;
    action["name"] = "GotoDst";
    action["id"] = 1;
    action["groupid"] = 1;

    // 添加参数
    action["params"] = nlohmann::json::array();

    // 目标位置
    nlohmann::json dst_param;
    dst_param["name"] = "dstLoc";
    dst_param["value"] = nlohmann::json::array();
    nlohmann::json point;
    point["x_lat"] = x;
    point["y_lon"] = y;
    point["z_alt"] = z;
    dst_param["value"].push_back(point);
    action["params"].push_back(dst_param);

    // 点类型
    nlohmann::json pt_type;
    pt_type["name"] = "pointTag";
    pt_type["value"] = "loc";
    action["params"].push_back(pt_type);

    // 速度
    nlohmann::json spd_param;
    spd_param["name"] = "spd";
    spd_param["value"] = 5.0;
    action["params"].push_back(spd_param);

    // 到达距离
    nlohmann::json arv_param;
    arv_param["name"] = "arvDis";
    arv_param["value"] = 2.0;
    action["params"].push_back(arv_param);

    goto_stage["actions"].push_back(action);
    mission_json["stage"].push_back(goto_stage);

    std_msgs::msg::String msg;
    msg.data = mission_json.dump();
    mission_json_pub_->publish(msg);

    RCLCPP_INFO(get_logger(), "Sent GotoDst mission JSON: target=(%.1f, %.1f, %.1f)", x, y, z);
  }

  void sendAutoTraceMission() {
    nlohmann::json mission_json;
    mission_json["stage"] = nlohmann::json::array();

    nlohmann::json trace_stage;
    trace_stage["name"] = "AutoTrace";
    trace_stage["sn"] = 3;
    trace_stage["cmd"] = "set";

    trace_stage["actions"] = nlohmann::json::array();
    nlohmann::json action;
    action["name"] = "AutoTrace";
    action["id"] = 1;
    action["groupid"] = 1;

    trace_stage["actions"].push_back(action);
    mission_json["stage"].push_back(trace_stage);

    std_msgs::msg::String msg;
    msg.data = mission_json.dump();
    mission_json_pub_->publish(msg);

    RCLCPP_INFO(get_logger(), "Sent AutoTrace mission JSON");
  }

  void sendLandMission() {
    nlohmann::json mission_json;
    mission_json["stage"] = nlohmann::json::array();

    nlohmann::json land_stage;
    land_stage["name"] = "Land";
    land_stage["sn"] = 4;
    land_stage["cmd"] = "set";

    land_stage["actions"] = nlohmann::json::array();
    nlohmann::json action;
    action["name"] = "Land";
    action["id"] = 1;
    action["groupid"] = 1;

    land_stage["actions"].push_back(action);
    mission_json["stage"].push_back(land_stage);

    std_msgs::msg::String msg;
    msg.data = mission_json.dump();
    mission_json_pub_->publish(msg);

    RCLCPP_INFO(get_logger(), "Sent Land mission JSON");
  }

  void sendStopMission(int stage_sn) {
    nlohmann::json mission_json;
    mission_json["stage"] = nlohmann::json::array();

    nlohmann::json stop_stage;
    stop_stage["name"] = "Stop";
    stop_stage["sn"] = stage_sn;
    stop_stage["cmd"] = "del";
    stop_stage["actions"] = nlohmann::json::array();

    mission_json["stage"].push_back(stop_stage);

    std_msgs::msg::String msg;
    msg.data = mission_json.dump();
    mission_json_pub_->publish(msg);

    RCLCPP_INFO(get_logger(), "Sent Stop mission for stage: %d", stage_sn);
  }

  void runBasicTest() {
    RCLCPP_INFO(get_logger(), "Starting basic behavior tree test sequence...");

    // 1. 设置Home点
    std::this_thread::sleep_for(1s);
    sendSetHomeCommand();

    // 2. 起飞任务
    std::this_thread::sleep_for(2s);
    sendTakeoffMission();

    // 3. 等待起飞完成，然后飞到目标点
    std::this_thread::sleep_for(5s);
    sendGotoDstMission(100.0, 50.0, -30.0);

    // 4. 等待到达，然后启动自动跟踪
    std::this_thread::sleep_for(8s);
    sendAutoTraceMission();

    // 5. 运行一段时间后降落
    std::this_thread::sleep_for(10s);
    sendLandMission();

    RCLCPP_INFO(get_logger(), "Basic test sequence completed");
  }

  void runInteractiveTest() {
    RCLCPP_INFO(get_logger(), "Interactive test mode. Commands:");
    RCLCPP_INFO(get_logger(), "  1: Set Home");
    RCLCPP_INFO(get_logger(), "  2: TakeOff");
    RCLCPP_INFO(get_logger(), "  3: Goto Destination");
    RCLCPP_INFO(get_logger(), "  4: Auto Trace");
    RCLCPP_INFO(get_logger(), "  5: Land");
    RCLCPP_INFO(get_logger(), "  9: Stop Mission");
    RCLCPP_INFO(get_logger(), "  0: Exit");

    int choice;
    while (rclcpp::ok()) {
      std::cout << "Enter command: ";
      if (!(std::cin >> choice)) {
        std::cin.clear();
        std::cin.ignore(10000, '\n');
        continue;
      }

      switch (choice) {
        case 1:
          sendSetHomeCommand();
          break;
        case 2:
          sendTakeoffMission();
          break;
        case 3:
          sendGotoDstMission();
          break;
        case 4:
          sendAutoTraceMission();
          break;
        case 5:
          sendLandMission();
          break;
        case 9:
          sendStopMission(1);
          break;
        case 0:
          return;
        default:
          RCLCPP_WARN(get_logger(), "Invalid command");
          break;
      }

      std::this_thread::sleep_for(100ms);
    }
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto client = std::make_shared<TestClient>();

  bool interactive = false;
  if (argc > 1 && std::string(argv[1]) == "--interactive") {
    interactive = true;
  }

  // 等待节点启动
  std::this_thread::sleep_for(1s);

  if (interactive) {
    client->runInteractiveTest();
  } else {
    // 运行自动测试
    std::thread test_thread([client]() {
      client->runBasicTest();
    });

    rclcpp::spin(client);
    test_thread.join();
  }

  rclcpp::shutdown();
  return 0;
}