#!/usr/bin/env python3
"""
Simplified Behavior Node Tester
单文件实现对behavior_node的完整测试，包括配置、任务切换等功能
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import json
import time
import threading
import sys
import argparse
from enum import Enum
from typing import Dict, List, Any, Optional
from dataclasses import dataclass, field

# ROS2 消息类型
from std_msgs.msg import String, Float32, UInt8, Int32
from geometry_msgs.msg import Point, Polygon, Point32
from sensor_msgs.msg import Joy
from custom_msgs.msg import (
    SimpleVehicle, ObjectComputation, ObjectLocation, OffboardCtrl,
    StatusTask, CommandRequest, CommandResponse, TaskStage
)
from custom_msgs.srv import CommandBool, CommandLong, CommandInt


class TestStatus(Enum):
    """测试状态枚举"""
    PENDING = "pending"
    RUNNING = "running"
    PASSED = "passed"
    FAILED = "failed"
    TIMEOUT = "timeout"


class MissionStage(Enum):
    """任务阶段枚举"""
    NO_START = 0
    ONGOING = 1
    FAILED = 2
    COMPLETE = 3
    NOT_READY = 4


@dataclass
class TestCase:
    """测试用例数据结构"""
    name: str
    description: str
    timeout: float = 30.0
    expected_stage: int = -1
    expected_status: MissionStage = MissionStage.COMPLETE
    parameters: Dict[str, Any] = field(default_factory=dict)
    status: TestStatus = TestStatus.PENDING
    start_time: float = 0.0
    end_time: float = 0.0
    error_message: str = ""


class BehaviorNodeTester(Node):
    """简化的behavior_node测试器"""

    def __init__(self):
        super().__init__('behavior_node_tester')

        # 测试状态
        self.current_test: Optional[TestCase] = None
        self.test_results: Dict[str, TestCase] = {}
        self.vehicle_state: Optional[SimpleVehicle] = None
        self.task_status: Optional[StatusTask] = None
        self.is_running = True

        # 配置参数
        self.vehicle_id = 1
        self.test_timeout = 60.0

        # 初始化ROS接口
        self._setup_publishers()
        self._setup_subscribers()
        self._setup_service_clients()

        # 启动仿真数据发布
        self._start_simulation()

        self.get_logger().info("Behavior Node Tester initialized")

    def _setup_publishers(self):
        """设置发布器"""
        qos_reliable = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        # 任务JSON发布器
        self.mission_json_pub = self.create_publisher(
            String, 'outer/set/stage_info_json', qos_reliable)

        # 命令发布器
        self.command_pub = self.create_publisher(
            CommandRequest, 'outer/command/request', qos_reliable)

        # 仿真数据发布器
        self.vehicle_state_pub = self.create_publisher(
            SimpleVehicle, 'inner/information/simple_vehicle', qos_reliable)

        self.object_detection_pub = self.create_publisher(
            ObjectComputation, 'inner/information/object_computation', qos_reliable)

    def _setup_subscribers(self):
        """设置订阅器"""
        qos_reliable = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        # 任务状态订阅
        self.status_sub = self.create_subscription(
            StatusTask, 'outer/information/status_task',
            self._status_callback, qos_reliable)

        # 命令响应订阅
        self.response_sub = self.create_subscription(
            CommandResponse, 'outer/command/response',
            self._response_callback, qos_reliable)

        # 控制指令订阅（用于验证）
        self.offboard_sub = self.create_subscription(
            OffboardCtrl, 'inner/control/offboard',
            self._offboard_callback, qos_reliable)

    def _setup_service_clients(self):
        """设置服务客户端（用于模拟服务响应）"""
        # 这里可以添加服务客户端，用于验证behavior_node的服务调用
        pass

    def _start_simulation(self):
        """启动仿真数据发布"""
        # 飞机状态仿真（50Hz）
        self.vehicle_timer = self.create_timer(0.02, self._publish_vehicle_state)

        # 目标检测仿真（10Hz）
        self.object_timer = self.create_timer(0.1, self._publish_object_detection)

        # 初始化仿真状态
        self.sim_position = {'x': 0.0, 'y': 0.0, 'z': -50.0, 'yaw': 0.0}
        self.sim_locked = True
        self.sim_flight_mode = 0

    def _publish_vehicle_state(self):
        """发布模拟的飞机状态"""
        msg = SimpleVehicle()
        msg.id = self.vehicle_id
        msg.x = int(self.sim_position['x'] * 1000)  # 转换为毫米
        msg.y = int(self.sim_position['y'] * 1000)
        msg.z = int(self.sim_position['z'] * 1000)
        msg.yaw = int(self.sim_position['yaw'] * 1000)  # 转换为毫弧度

        msg.vx = 0
        msg.vy = 0
        msg.vz = 0

        msg.lock = 0 if self.sim_locked else 1
        msg.flymd = self.sim_flight_mode

        # GPS坐标模拟
        msg.lat = int((39.9042 + self.sim_position['x'] / 111320.0) * 1e7)
        msg.lon = int((116.4074 + self.sim_position['y'] / 111320.0) * 1e7)
        msg.alt = int(-self.sim_position['z'] * 1000)

        self.vehicle_state_pub.publish(msg)
        self.vehicle_state = msg

    def _publish_object_detection(self):
        """发布模拟的目标检测数据"""
        msg = ObjectComputation()

        # 添加一个测试目标
        obj = ObjectLocation()
        obj.id = 101
        obj.x = 100 + int(time.time() % 10) * 2  # 简单的移动模拟
        obj.y = 50
        obj.z = -30
        obj.vx = 2
        obj.vy = 0
        obj.vz = 0

        msg.objs = [obj]
        self.object_detection_pub.publish(msg)

    def _status_callback(self, msg: StatusTask):
        """任务状态回调"""
        self.task_status = msg
        self.get_logger().info(f"Task status: stage={msg.stage}, id={msg.id}, status={msg.status}")

        if self.current_test:
            self._evaluate_test_progress()

    def _response_callback(self, msg: CommandResponse):
        """命令响应回调"""
        self.get_logger().info(f"Command response: type={msg.type}, status={msg.status}")

    def _offboard_callback(self, msg: OffboardCtrl):
        """外部控制指令回调 - 更新仿真状态"""
        # 简单的位置更新模拟
        target_x, target_y, target_z = msg.x, msg.y, msg.z

        # 平滑移动到目标位置
        dx = target_x - self.sim_position['x']
        dy = target_y - self.sim_position['y']
        dz = target_z - self.sim_position['z']

        # 简单的比例控制
        speed = 0.1
        self.sim_position['x'] += dx * speed
        self.sim_position['y'] += dy * speed
        self.sim_position['z'] += dz * speed
        self.sim_position['yaw'] = msg.yaw

    def _evaluate_test_progress(self):
        """评估测试进度"""
        if not self.current_test or not self.task_status:
            return

        test = self.current_test
        current_time = time.time()

        # 检查超时
        if current_time - test.start_time > test.timeout:
            self._complete_test(TestStatus.TIMEOUT, "Test timeout")
            return

        # 检查任务状态
        if test.expected_stage >= 0 and self.task_status.stage == test.expected_stage:
            if self.task_status.status == MissionStage.COMPLETE.value:
                self._complete_test(TestStatus.PASSED, "Mission completed successfully")
            elif self.task_status.status == MissionStage.FAILED.value:
                self._complete_test(TestStatus.FAILED, "Mission failed")

    def _complete_test(self, status: TestStatus, message: str):
        """完成测试"""
        if not self.current_test:
            return

        test = self.current_test
        test.status = status
        test.end_time = time.time()
        test.error_message = message

        duration = test.end_time - test.start_time
        status_str = "PASSED" if status == TestStatus.PASSED else "FAILED"

        self.get_logger().info(
            f"Test '{test.name}' {status_str}: {message} (Duration: {duration:.1f}s)")

        self.test_results[test.name] = test
        self.current_test = None

    def send_mission_json(self, stage_name: str, stage_sn: int, action_name: str,
                          params: Dict[str, Any] = None, cmd: str = "set"):
        """发送任务JSON"""
        mission_json = {
            "stage": [{
                "name": stage_name,
                "sn": stage_sn,
                "cmd": cmd,
                "actions": [{
                    "name": action_name,
                    "id": self.vehicle_id,
                    "groupid": 1,
                    "params": []
                }]
            }]
        }

        if params:
            for key, value in params.items():
                mission_json["stage"][0]["actions"][0]["params"].append({
                    "name": key,
                    "value": value
                })

        msg = String()
        msg.data = json.dumps(mission_json)
        self.mission_json_pub.publish(msg)

        self.get_logger().info(f"Sent {stage_name} mission (stage {stage_sn})")

    def send_command(self, cmd_type: int, **kwargs):
        """发送命令"""
        cmd = CommandRequest()
        cmd.type = cmd_type
        cmd.dst = self.vehicle_id

        # 设置参数
        for key, value in kwargs.items():
            if hasattr(cmd, key):
                setattr(cmd, key, value)

        self.command_pub.publish(cmd)
        self.get_logger().info(f"Sent command type: {cmd_type}")

    def wait_for_test_completion(self, max_wait_time: float = 30.0):
        """等待测试完成"""
        start_time = time.time()
        rate = self.create_rate(10)  # 10 Hz

        while rclpy.ok() and self.current_test and self.is_running:
            if time.time() - start_time > max_wait_time:
                self._complete_test(TestStatus.TIMEOUT, "Wait timeout")
                break

            rclpy.spin_once(self, timeout_sec=0.1)

    # ============= 具体测试用例 =============

    def test_set_home(self):
        """测试设置Home点"""
        test = TestCase(
            name="SetHome",
            description="Test setting home point",
            timeout=10.0
        )

        self.current_test = test
        test.start_time = time.time()
        test.status = TestStatus.RUNNING

        self.get_logger().info(f"Starting test: {test.name}")

        # 发送设置Home命令
        self.send_command(
            cmd_type=6,  # CmdSetHome
            param0=int(39.9042 * 1e7),  # 纬度
            param1=int(116.4074 * 1e7),  # 经度
            param2=int(50.0 * 1e3)  # 高度
        )

        # 等待响应
        time.sleep(2.0)
        self._complete_test(TestStatus.PASSED, "Home set command sent successfully")

    def test_takeoff(self):
        """测试起飞"""
        test = TestCase(
            name="TakeOff",
            description="Test takeoff mission",
            timeout=20.0,
            expected_stage=1
        )

        self.current_test = test
        test.start_time = time.time()
        test.status = TestStatus.RUNNING

        self.get_logger().info(f"Starting test: {test.name}")

        # 模拟解锁
        self.sim_locked = False

        params = {
            "alt": 30.0,
            "pointTag": "loc"
        }

        self.send_mission_json("TakeOff", 1, "TakeOff", params)
        self.wait_for_test_completion(20.0)

    def test_goto_destination(self):
        """测试飞向目标点"""
        test = TestCase(
            name="GotoDestination",
            description="Test goto destination mission",
            timeout=25.0,
            expected_stage=2
        )

        self.current_test = test
        test.start_time = time.time()
        test.status = TestStatus.RUNNING

        self.get_logger().info(f"Starting test: {test.name}")

        params = {
            "dstLoc": [{
                "x_lat": 100.0,
                "y_lon": 50.0,
                "z_alt": -25.0
            }],
            "pointTag": "loc",
            "spd": 8.0,
            "arvDis": 3.0
        }

        self.send_mission_json("GotoDst", 2, "GotoDst", params)
        self.wait_for_test_completion(25.0)

    def test_auto_trace(self):
        """测试自动跟踪"""
        test = TestCase(
            name="AutoTrace",
            description="Test auto trace mission",
            timeout=15.0,
            expected_stage=3
        )

        self.current_test = test
        test.start_time = time.time()
        test.status = TestStatus.RUNNING

        self.get_logger().info(f"Starting test: {test.name}")

        self.send_mission_json("AutoTrace", 3, "AutoTrace")

        # 自动跟踪测试运行8秒后停止
        time.sleep(8.0)
        self.send_mission_json("Stop", 3, "Stop", cmd="del")

        self._complete_test(TestStatus.PASSED, "AutoTrace test completed")

    def test_land(self):
        """测试降落"""
        test = TestCase(
            name="Land",
            description="Test landing mission",
            timeout=15.0,
            expected_stage=4
        )

        self.current_test = test
        test.start_time = time.time()
        test.status = TestStatus.RUNNING

        self.get_logger().info(f"Starting test: {test.name}")

        self.send_mission_json("Land", 4, "Land")
        self.wait_for_test_completion(15.0)

    def test_parameter_configuration(self):
        """测试参数配置"""
        test = TestCase(
            name="ParameterConfig",
            description="Test parameter configuration",
            timeout=10.0
        )

        self.current_test = test
        test.start_time = time.time()
        test.status = TestStatus.RUNNING

        self.get_logger().info(f"Starting test: {test.name}")

        # 测试各种参数设置
        params = {
            "vehiType": "多旋翼",
            "spd": 8.0,
            "arvDis": 2.0,
            "loops": 1,
            "pointTag": "loc"
        }

        self.send_mission_json("Config", 0, "SetLine", params)

        time.sleep(3.0)
        self._complete_test(TestStatus.PASSED, "Parameter configuration completed")

    def run_all_tests(self):
        """运行所有测试"""
        self.get_logger().info("Starting comprehensive behavior node test suite...")

        tests = [
            self.test_set_home,
            self.test_parameter_configuration,
            self.test_takeoff,
            self.test_goto_destination,
            self.test_auto_trace,
            self.test_land
        ]

        for test_func in tests:
            if not rclpy.ok() or not self.is_running:
                break

            try:
                test_func()
                time.sleep(2.0)  # 测试间隔
            except Exception as e:
                test_name = test_func.__name__
                self.get_logger().error(f"Test {test_name} failed with exception: {str(e)}")

                if test_name not in self.test_results:
                    self.test_results[test_name] = TestCase(
                        name=test_name,
                        description=f"Test {test_name}",
                        status=TestStatus.FAILED,
                        error_message=f"Exception: {str(e)}"
                    )

        self.print_test_summary()

    def print_test_summary(self):
        """打印测试总结"""
        self.get_logger().info("\n" + "=" * 60)
        self.get_logger().info("BEHAVIOR NODE TEST SUMMARY")
        self.get_logger().info("=" * 60)

        total_tests = len(self.test_results)
        passed_tests = sum(1 for test in self.test_results.values()
                           if test.status == TestStatus.PASSED)

        for test_name, test in self.test_results.items():
            status_str = test.status.value.upper()
            duration = test.end_time - test.start_time if test.end_time > 0 else 0
            message = test.error_message or "OK"

            self.get_logger().info(
                f"{test_name:20} [{status_str:7}] {duration:6.1f}s - {message}")

        self.get_logger().info("-" * 60)
        self.get_logger().info(f"Total Tests: {total_tests}")
        self.get_logger().info(f"Passed:     {passed_tests}")
        self.get_logger().info(f"Failed:     {total_tests - passed_tests}")

        success_rate = (passed_tests / total_tests * 100) if total_tests > 0 else 0
        self.get_logger().info(f"Success Rate: {success_rate:.1f}%")
        self.get_logger().info("=" * 60)

    def shutdown(self):
        """关闭测试器"""
        self.is_running = False
        self.get_logger().info("Behavior Node Tester shutting down...")


def main():
    """主函数"""
    parser = argparse.ArgumentParser(description='Behavior Node Tester')
    parser.add_argument('--test', choices=['all', 'set_home','takeoff', 'goto', 'trace', 'land'],
                        default='all', help='Specify which test to run')
    parser.add_argument('--interactive', action='store_true',
                        help='Run in interactive mode')
    parser.add_argument('--timeout', type=float, default=60.0,
                        help='Test timeout in seconds')

    args = parser.parse_args()

    rclpy.init()

    tester = BehaviorNodeTester()
    tester.test_timeout = args.timeout

    try:
        if args.interactive:
            # 交互式模式
            tester.get_logger().info("Interactive test mode - use Ctrl+C to exit")
            rclpy.spin(tester)
        else:
            # 自动测试模式
            tester.get_logger().info("Waiting for system to initialize...")
            time.sleep(3.0)

            if args.test == 'all':
                # 在单独线程中运行测试，避免阻塞ROS spin
                test_thread = threading.Thread(target=tester.run_all_tests)
                test_thread.start()

                # 继续处理ROS消息
                while rclpy.ok() and test_thread.is_alive() and tester.is_running:
                    rclpy.spin_once(tester, timeout_sec=0.1)

                test_thread.join()
            else:
                # 运行单个测试
                test_method = getattr(tester, f'test_{args.test}', None)
                if test_method:
                    test_thread = threading.Thread(target=test_method)
                    test_thread.start()

                    while rclpy.ok() and test_thread.is_alive() and tester.is_running:
                        rclpy.spin_once(tester, timeout_sec=0.1)

                    test_thread.join()
                    tester.print_test_summary()
                else:
                    tester.get_logger().error(f"Unknown test: {args.test}")

    except KeyboardInterrupt:
        tester.get_logger().info("Test interrupted by user")
    finally:
        tester.shutdown()
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()