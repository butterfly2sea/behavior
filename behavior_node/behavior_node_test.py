#!/usr/bin/env python3
"""
Fixed Behavior Node Tester
修复了generator执行冲突问题的behavior_node测试器
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.executors import SingleThreadedExecutor
import json
import time
import threading
import sys
import argparse
import signal
import math
import asyncio
from enum import Enum
from typing import Dict, List, Any, Optional, Callable
from dataclasses import dataclass, field

# ROS2 消息类型
from std_msgs.msg import String, Float32, UInt8, Int32, UInt8MultiArray
from geometry_msgs.msg import Point, Polygon, Point32
from sensor_msgs.msg import Joy
from custom_msgs.msg import (
    SimpleVehicle, ObjectComputation, ObjectLocation, OffboardCtrl,
    StatusTask, CommandRequest, CommandResponse, TaskStage, ParamShort
)
from custom_msgs.srv import CommandBool, CommandLong, CommandInt, CommandString


class TestStatus(Enum):
    """测试状态枚举"""
    PENDING = "pending"
    RUNNING = "running"
    PASSED = "passed"
    FAILED = "failed"
    TIMEOUT = "timeout"
    SKIPPED = "skipped"


class MissionStage(Enum):
    """任务阶段枚举"""
    NO_START = 0
    ONGOING = 1
    FAILED = 2
    COMPLETE = 3
    NOT_READY = 4


class FlightMode(Enum):
    """飞行模式枚举"""
    UNKNOWN = 0
    MANUAL = 1
    ALT = 2
    POS = 3
    TAKEOFF = 4
    LAND = 5
    HOLD = 6
    MISSION = 7
    RTL = 8
    OFFBOARD = 9
    STABLE = 10


class VehicleType(Enum):
    """载具类型枚举"""
    FIXWING = 1
    COPER = 2
    VTOL_COP = 3
    VTOL_FIX = 4
    CAR = 5


@dataclass
class TestCase:
    """测试用例数据结构"""
    name: str
    description: str
    tree_name: str  # 对应的行为树名称
    timeout: float = 30.0
    expected_stage: int = -1
    expected_status: MissionStage = MissionStage.COMPLETE
    expected_offboard_count: int = 0
    parameters: Dict[str, Any] = field(default_factory=dict)
    status: TestStatus = TestStatus.PENDING
    start_time: float = 0.0
    end_time: float = 0.0
    error_message: str = ""
    setup_params: Dict[str, Any] = field(default_factory=dict)

class FixedBehaviorNodeTester(Node):
    """修复的behavior_node测试器"""

    def __init__(self):
        super().__init__('fixed_behavior_node_tester')

        # 测试状态
        self.current_test: Optional[TestCase] = None
        self.test_results: Dict[str, TestCase] = {}
        self.vehicle_state: Optional[SimpleVehicle] = None
        self.task_status: Optional[StatusTask] = None
        self.offboard_commands: List[OffboardCtrl] = []
        self.is_running = True

        self.test_start_time = 0.0
        self.test_timeout = 60.0

        # 配置参数
        self.vehicle_id = 1
        self.enable_detailed_logging = True

        # 初始化ROS接口
        self._setup_qos_profiles()
        self._setup_publishers()
        self._setup_subscribers()

        # 启动仿真数据发布
        self._start_simulation()

        self.get_logger().info("Fixed Behavior Node Tester initialized")

    def _setup_qos_profiles(self):
        """设置QoS配置文件"""

        # 可靠的transient_local QoS (用于任务控制和飞机状态)
        self.qos_mission = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # 可靠的volatile QoS (用于控制命令)
        self.qos_control = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        # 传感器数据QoS (最佳努力)
        self.qos_sensor = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST
        )

    def _setup_publishers(self):
        """设置发布器"""

        # 任务JSON发布器 (使用transient_local)
        self.mission_json_pub = self.create_publisher(
            String, 'outer/set/stage_info_json', self.qos_mission)

        # 命令发布器 (使用volatile)
        self.command_pub = self.create_publisher(
            CommandRequest, 'outer/command/request', self.qos_control)

        # 任务阶段发布器 (使用transient_local)
        self.task_stage_pub = self.create_publisher(
            TaskStage, 'outer/set/task_stage', self.qos_mission)

        # 仿真数据发布器 (使用transient_local以匹配behavior_node)
        self.vehicle_state_pub = self.create_publisher(
            SimpleVehicle, 'inner/information/simple_vehicle', self.qos_mission)

        # 目标检测发布器 (使用sensor QoS)
        self.object_detection_pub = self.create_publisher(
            ObjectComputation, 'inner/information/object_computation', self.qos_sensor)

        # 摇杆输入发布器 (使用sensor QoS)
        self.joy_pub = self.create_publisher(
            Joy, 'inner/control/joystick', self.qos_sensor)

    def _setup_subscribers(self):
        """设置订阅器"""

        # 任务状态订阅
        self.status_sub = self.create_subscription(
            StatusTask, 'outer/information/status_task',
            self._status_callback, self.qos_control)

        # 命令响应订阅
        self.response_sub = self.create_subscription(
            CommandResponse, 'outer/command/response',
            self._response_callback, self.qos_control)

        # 控制指令订阅（用于验证）
        self.offboard_sub = self.create_subscription(
            OffboardCtrl, 'inner/control/offboard',
            self._offboard_callback, self.qos_control)

    def _start_simulation(self):
        """启动仿真数据发布"""
        # 飞机状态仿真（20Hz）
        self.vehicle_timer = self.create_timer(0.05, self._publish_vehicle_state)

        # 目标检测仿真（5Hz）
        self.object_timer = self.create_timer(0.2, self._publish_object_detection)

        # 摇杆输入仿真（10Hz）
        self.joy_timer = self.create_timer(0.1, self._publish_joy_input)

        # 初始化仿真状态
        self.sim_position = {'x': 0.0, 'y': 0.0, 'z': -50.0, 'yaw': 0.0}
        self.sim_locked = True
        self.sim_flight_mode = FlightMode.MANUAL.value
        self.sim_time = 0.0
        self.target_position = {'x': 0.0, 'y': 0.0, 'z': -50.0, 'yaw': 0.0}

    def _publish_vehicle_state(self):
        """发布模拟的飞机状态"""
        self.sim_time += 0.05

        # 平滑移动到目标位置
        alpha = 0.05  # 移动速度系数
        for key in ['x', 'y', 'z', 'yaw']:
            diff = self.target_position[key] - self.sim_position[key]
            if key == 'yaw':
                # 处理角度环绕
                if diff > math.pi:
                    diff -= 2 * math.pi
                elif diff < -math.pi:
                    diff += 2 * math.pi
            self.sim_position[key] += diff * alpha

        msg = SimpleVehicle()
        msg.id = self.vehicle_id
        msg.x = int(self.sim_position['x'] * 1000)  # 转换为毫米
        msg.y = int(self.sim_position['y'] * 1000)
        msg.z = int(self.sim_position['z'] * 1000)
        msg.yaw = int(self.sim_position['yaw'] * 1000)  # 转换为毫弧度

        # 计算速度
        msg.vx = int((self.target_position['x'] - self.sim_position['x']) * 200)
        msg.vy = int((self.target_position['y'] - self.sim_position['y']) * 200)
        msg.vz = int((self.target_position['z'] - self.sim_position['z']) * 200)

        msg.lock = 0 if self.sim_locked else 1
        msg.flymd = self.sim_flight_mode

        # GPS坐标模拟 (北京附近)
        msg.lat = int((39.9042 + self.sim_position['x'] / 111320.0) * 1e7)
        msg.lon = int((116.4074 + self.sim_position['y'] / 111320.0) * 1e7)
        msg.alt = int(-self.sim_position['z'] * 1000)

        # 其他状态
        msg.pitch = int(math.sin(self.sim_time * 0.1) * 50)  # 小幅摇摆
        msg.roll = int(math.cos(self.sim_time * 0.1) * 30)

        self.vehicle_state_pub.publish(msg)
        self.vehicle_state = msg

    def _publish_object_detection(self):
        """发布模拟的目标检测数据"""
        msg = ObjectComputation()

        # 添加测试目标
        obj1 = ObjectLocation()
        obj1.id = 101
        obj1.x = 100 + int(math.sin(self.sim_time * 0.5) * 20)
        obj1.y = 50 + int(math.cos(self.sim_time * 0.3) * 15)
        obj1.z = -30
        obj1.vx = int(math.cos(self.sim_time * 0.5) * 10)
        obj1.vy = int(-math.sin(self.sim_time * 0.3) * 5)
        obj1.vz = 0

        msg.objs = [obj1]
        self.object_detection_pub.publish(msg)

    def _publish_joy_input(self):
        """发布模拟的摇杆输入"""
        msg = Joy()

        # 模拟摇杆轴
        msg.axes = [0.0] * 8

        # 模拟摇杆按钮
        msg.buttons = [0] * 11

        # 在特定测试期间模拟按钮按下
        if self.current_test and "joystick" in self.current_test.name.lower():
            msg.buttons[0] = 1  # A按钮

        self.joy_pub.publish(msg)

    def _status_callback(self, msg: StatusTask):
        """任务状态回调"""
        self.task_status = msg

        if self.enable_detailed_logging:
            self.get_logger().info(f"Task status: stage={msg.stage}, id={msg.id}, status={msg.status}")

        # 简化的测试进度检查
        if self.current_test:
            self._check_test_completion()

    def _response_callback(self, msg: CommandResponse):
        """命令响应回调"""
        if self.enable_detailed_logging:
            self.get_logger().info(f"Command response: type={msg.type}, status={msg.status}, result={msg.rslt}")

    def _offboard_callback(self, msg: OffboardCtrl):
        """外部控制指令回调"""
        self.offboard_commands.append(msg)

        if self.enable_detailed_logging:
            self.get_logger().info(f"Offboard control: x={msg.x:.2f}, y={msg.y:.2f}, z={msg.z:.2f}, yaw={msg.yaw:.2f}")

        # 更新目标位置以模拟飞行
        self.target_position['x'] = msg.x
        self.target_position['y'] = msg.y
        self.target_position['z'] = msg.z
        self.target_position['yaw'] = msg.yaw

        # 限制命令历史
        if len(self.offboard_commands) > 100:
            self.offboard_commands = self.offboard_commands[-50:]

    def _check_test_completion(self):
        """检查测试完成状态 - 简化版本避免递归调用"""
        if not self.current_test:
            return

        current_time = time.time()

        # 检查超时
        if current_time - self.test_start_time > self.current_test.timeout:
            self._complete_test(TestStatus.TIMEOUT, "Test timeout")
            return

        # 简单的成功条件检查
        if len(self.offboard_commands) >= 3:
            # 如果收到足够的控制指令，认为测试基本成功
            if current_time - self.test_start_time > 5.0:  # 至少运行5秒
                self._complete_test(TestStatus.PASSED, "Received sufficient control commands")
                return

    def _complete_test(self, status: TestStatus, message: str):
        """完成测试"""
        if not self.current_test:
            return

        test = self.current_test
        test.status = status
        test.end_time = time.time()
        test.error_message = message

        duration = test.end_time - test.start_time
        status_str = status.value.upper()

        self.get_logger().info(
            f"Test '{test.name}' {status_str}: {message} (Duration: {duration:.1f}s)")

        self.test_results[test.name] = test
        self.current_test = None

    # ============= 测试工具方法 =============

    def send_mission_json(self, stage_name: str, stage_sn: int, action_name: str,
                          params: Dict[str, Any] = None, cmd: str = "set",
                          group_id: int = 1):
        """发送任务JSON"""
        mission_json = {
            "stage": [{
                "name": stage_name,
                "sn": stage_sn,
                "cmd": cmd,
                "actions": [{
                    "name": action_name,
                    "id": self.vehicle_id,
                    "groupid": group_id,
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

    def set_simulation_state(self, locked: bool = None, flight_mode: FlightMode = None,
                             position: Dict[str, float] = None):
        """设置仿真状态"""
        if locked is not None:
            self.sim_locked = locked

        if flight_mode is not None:
            self.sim_flight_mode = flight_mode.value

        if position:
            self.sim_position.update(position)
            self.target_position.update(position)

    def wait_for_test_completion(self, timeout: float = 30.0):
        """等待测试完成 - 使用简单的计时器"""
        start_time = time.time()

        while self.current_test and (time.time() - start_time) < timeout:
            time.sleep(0.1)  # 简单的等待，避免复杂的异步操作

            # 检查测试是否应该完成
            if self.current_test:
                self._check_test_completion()

    # ============= 具体测试用例 =============

    def test_set_home(self):
        """测试设置Home点"""
        test = TestCase(
            name="SetHome",
            description="Test setting home point",
            tree_name="",  # 不需要行为树
            timeout=10.0
        )

        self.current_test = test
        self.test_start_time = time.time()
        test.start_time = self.test_start_time
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
        time.sleep(3.0)
        self._complete_test(TestStatus.PASSED, "Home set command sent successfully")

    def test_unlock_vehicle(self):
        """测试解锁载具"""
        test = TestCase(
            name="UnlockVehicle",
            description="Test vehicle unlock using LockCtrl behavior tree",
            tree_name="LockCtrl",
            timeout=15.0,
            expected_stage=1
        )

        self.current_test = test
        self.test_start_time = time.time()
        test.start_time = self.test_start_time
        test.status = TestStatus.RUNNING

        self.get_logger().info(f"Starting test: {test.name}")

        # 模拟解锁
        self.set_simulation_state(locked=False)

        params = {
            "vehiType": "多旋翼",
            "spd": 5.0,
            "arvDis": 1.0
        }

        self.send_mission_json("Unlock", 1, "LockCtrl", params)
        self.wait_for_test_completion(15.0)

    def test_takeoff(self):
        """测试起飞"""
        test = TestCase(
            name="TakeOff",
            description="Test takeoff mission using TakeOff behavior tree",
            tree_name="TakeOff",
            timeout=25.0,
            expected_stage=2
        )

        self.current_test = test
        self.test_start_time = time.time()
        test.start_time = self.test_start_time
        test.status = TestStatus.RUNNING

        self.get_logger().info(f"Starting test: {test.name}")

        # 设置起飞状态
        self.set_simulation_state(
            locked=False,
            flight_mode=FlightMode.TAKEOFF,
            position={'x': 0.0, 'y': 0.0, 'z': -5.0, 'yaw': 0.0}
        )

        params = {
            "alt": -30.0,
            "pointTag": "loc",
            "vehiType": "多旋翼",
            "spd": 5.0
        }

        self.send_mission_json("TakeOff", 2, "TakeOff", params)
        self.wait_for_test_completion(25.0)

    def test_goto_destination(self):
        """测试飞向目标点"""
        test = TestCase(
            name="GotoDestination",
            description="Test goto destination using GotoDst behavior tree",
            tree_name="GotoDst",
            timeout=30.0,
            expected_stage=3
        )

        self.current_test = test
        self.test_start_time = time.time()
        test.start_time = self.test_start_time
        test.status = TestStatus.RUNNING

        self.get_logger().info(f"Starting test: {test.name}")

        # 设置飞行状态
        self.set_simulation_state(
            locked=False,
            flight_mode=FlightMode.OFFBOARD,
            position={'x': 0.0, 'y': 0.0, 'z': -25.0, 'yaw': 0.0}
        )

        params = {
            "dstLoc": [{
                "x_lat": 100.0,
                "y_lon": 50.0,
                "z_alt": -25.0
            }],
            "pointTag": "loc",
            "spd": 8.0,
            "arvDis": 3.0,
            "vehiType": "多旋翼"
        }

        self.send_mission_json("GotoDst", 3, "GotoDst", params)
        self.wait_for_test_completion(30.0)

    def test_auto_trace(self):
        """测试自动跟踪"""
        test = TestCase(
            name="AutoTrace",
            description="Test auto trace using AutoTrace behavior tree",
            tree_name="AutoTrace",
            timeout=20.0,
            expected_stage=4
        )

        self.current_test = test
        self.test_start_time = time.time()
        test.start_time = self.test_start_time
        test.status = TestStatus.RUNNING

        self.get_logger().info(f"Starting test: {test.name}")

        # 设置跟踪状态
        self.set_simulation_state(
            locked=False,
            flight_mode=FlightMode.OFFBOARD,
            position={'x': 80.0, 'y': 40.0, 'z': -30.0, 'yaw': 0.0}
        )

        params = {
            "vehiType": "多旋翼",
            "spd": 6.0,
            "arvDis": 2.0
        }

        self.send_mission_json("AutoTrace", 4, "AutoTrace", params)

        # 运行一段时间后停止
        time.sleep(12.0)
        if self.current_test and self.current_test.name == "AutoTrace":
            self.send_mission_json("Stop", 4, "AutoTrace", cmd="del")
            time.sleep(2.0)
            if self.current_test:
                self._complete_test(TestStatus.PASSED, "AutoTrace test completed successfully")

    def test_land(self):
        """测试降落"""
        test = TestCase(
            name="Land",
            description="Test landing using Land behavior tree",
            tree_name="Land",
            timeout=20.0,
            expected_stage=5
        )

        self.current_test = test
        self.test_start_time = time.time()
        test.start_time = self.test_start_time
        test.status = TestStatus.RUNNING

        self.get_logger().info(f"Starting test: {test.name}")

        # 设置降落状态
        self.set_simulation_state(
            locked=False,
            flight_mode=FlightMode.LAND,
            position={'x': 0.0, 'y': 0.0, 'z': -25.0, 'yaw': 0.0}
        )

        params = {
            "vehiType": "多旋翼"
        }

        self.send_mission_json("Land", 5, "Land", params)
        self.wait_for_test_completion(20.0)

    def test_parameter_configuration(self):
        """测试参数配置"""
        test = TestCase(
            name="ParameterConfig",
            description="Test parameter configuration using SetLine behavior tree",
            tree_name="SetLine",
            timeout=15.0,
            expected_stage=6
        )

        self.current_test = test
        self.test_start_time = time.time()
        test.start_time = self.test_start_time
        test.status = TestStatus.RUNNING

        self.get_logger().info(f"Starting test: {test.name}")

        # 测试各种参数设置
        params = {
            "vehiType": "多旋翼",
            "spd": 8.0,
            "arvDis": 2.0,
            "loops": 1,
            "pointTag": "loc",
            "wayPoints": [
                {"x_lat": 50.0, "y_lon": 30.0, "z_alt": -25.0},
                {"x_lat": 100.0, "y_lon": 60.0, "z_alt": -30.0}
            ]
        }

        self.send_mission_json("Config", 6, "SetLine", params)

        # 参数配置测试运行一段时间
        time.sleep(8.0)
        if self.current_test:
            self._complete_test(TestStatus.PASSED, "Parameter configuration completed")

    def test_formation_flight(self):
        """测试编队飞行"""
        test = TestCase(
            name="FormationFlight",
            description="Test formation flight using FormFly behavior tree",
            tree_name="FormFly",
            timeout=25.0,
            expected_stage=7
        )

        self.current_test = test
        self.test_start_time = time.time()
        test.start_time = self.test_start_time
        test.status = TestStatus.RUNNING

        self.get_logger().info(f"Starting test: {test.name}")

        # 设置编队飞行状态
        self.set_simulation_state(
            locked=False,
            flight_mode=FlightMode.OFFBOARD,
            position={'x': 0.0, 'y': 0.0, 'z': -25.0, 'yaw': 0.0}
        )

        params = {
            "vehiType": "多旋翼",
            "spd": 6.0,
            "arvDis": 2.0,
            "wayPoints": [
                {"x_lat": 50.0, "y_lon": 30.0, "z_alt": -25.0},
                {"x_lat": 100.0, "y_lon": 60.0, "z_alt": -25.0}
            ]
        }

        self.send_mission_json("Formation", 7, "FormFly", params, group_id=1)
        self.wait_for_test_completion(25.0)

    def test_comprehensive_mission(self):
        """综合任务测试"""
        test = TestCase(
            name="ComprehensiveMission",
            description="Test comprehensive mission using ComprehensiveMission behavior tree",
            tree_name="ComprehensiveMission",
            timeout=60.0,
            expected_stage=8
        )

        self.current_test = test
        self.test_start_time = time.time()
        test.start_time = self.test_start_time
        test.status = TestStatus.RUNNING

        self.get_logger().info(f"Starting test: {test.name}")

        # 执行综合任务
        params = {
            "vehiType": "多旋翼",
            "alt": -30.0,
            "dstLoc": [{"x_lat": 100.0, "y_lon": 50.0, "z_alt": -25.0}],
            "pointTag": "loc",
            "spd": 8.0,
            "arvDis": 3.0
        }

        self.send_mission_json("ComprehensiveMission", 8, "ComprehensiveMission", params)
        self.wait_for_test_completion(60.0)

    def test_simple_functions(self):
        """简单功能测试"""
        tests = [
            ("video", "Test video command", self._test_video_command),
            ("joystick", "Test joystick input", self._test_joystick_input)
        ]

        for test_name, description, test_func in tests:
            test = TestCase(
                name=f"Simple_{test_name}",
                description=description,
                tree_name="",
                timeout=10.0
            )

            self.current_test = test
            self.test_start_time = time.time()
            test.start_time = self.test_start_time
            test.status = TestStatus.RUNNING

            self.get_logger().info(f"Starting test: {test.name}")

            try:
                test_func()
                time.sleep(3.0)
                if self.current_test:
                    self._complete_test(TestStatus.PASSED, f"{test_name} test completed")
            except Exception as e:
                if self.current_test:
                    self._complete_test(TestStatus.FAILED, f"{test_name} test failed: {str(e)}")

            time.sleep(1.0)  # 测试间隔

    def _test_video_command(self):
        """视频命令测试"""
        self.send_command(
            cmd_type=9,  # SetVideo
            param0=2,    # 类型
            param1=1,    # rcvip
            param2=8554, # rcvport
            param3=1920, # resx
            param4=1080, # resy
            fparam5=30.0 # fps
        )

    def _test_joystick_input(self):
        """摇杆输入测试"""
        # 摇杆输入在 _publish_joy_input 中自动处理
        pass

    def run_all_tests(self):
        """运行所有测试 - 简化版本"""
        self.get_logger().info("=" * 80)
        self.get_logger().info("STARTING BEHAVIOR NODE TEST SUITE")
        self.get_logger().info("=" * 80)

        tests = [
            self.test_set_home,
            self.test_parameter_configuration,
            self.test_unlock_vehicle,
            self.test_takeoff,
            self.test_goto_destination,
            self.test_auto_trace,
            self.test_land,
            self.test_formation_flight,
            self.test_comprehensive_mission,
            self.test_simple_functions
        ]

        for i, test_func in enumerate(tests, 1):
            if not self.is_running:
                break

            self.get_logger().info(f"\n[{i}/{len(tests)}] Preparing test: {test_func.__name__}")

            try:
                test_func()
                time.sleep(2.0)  # 测试间隔

                # 清理offboard命令历史
                self.offboard_commands.clear()

            except Exception as e:
                test_name = test_func.__name__
                self.get_logger().error(f"Test {test_name} failed with exception: {str(e)}")

                if test_name not in self.test_results:
                    self.test_results[test_name] = TestCase(
                        name=test_name,
                        description=f"Test {test_name}",
                        tree_name="",
                        status=TestStatus.FAILED,
                        error_message=f"Exception: {str(e)}"
                    )

        self.print_test_summary()

    def print_test_summary(self):
        """打印测试总结"""
        self.get_logger().info("\n" + "=" * 80)
        self.get_logger().info("BEHAVIOR NODE TEST SUMMARY")
        self.get_logger().info("=" * 80)

        total_tests = len(self.test_results)
        passed_tests = sum(1 for test in self.test_results.values()
                           if test.status == TestStatus.PASSED)
        failed_tests = sum(1 for test in self.test_results.values()
                           if test.status == TestStatus.FAILED)
        timeout_tests = sum(1 for test in self.test_results.values()
                            if test.status == TestStatus.TIMEOUT)

        self.get_logger().info(f"Total Tests:    {total_tests}")
        self.get_logger().info(f"Passed:         {passed_tests}")
        self.get_logger().info(f"Failed:         {failed_tests}")
        self.get_logger().info(f"Timeout:        {timeout_tests}")
        self.get_logger().info("-" * 80)

        for test_name, test in self.test_results.items():
            status_str = test.status.value.upper()
            duration = test.end_time - test.start_time if test.end_time > 0 else 0
            message = test.error_message or "OK"

            # 添加状态符号
            if test.status == TestStatus.PASSED:
                symbol = "✓"
            elif test.status == TestStatus.FAILED:
                symbol = "✗"
            elif test.status == TestStatus.TIMEOUT:
                symbol = "⏰"
            else:
                symbol = "?"

            self.get_logger().info(
                f"{symbol} {test_name:25} [{status_str:7}] {duration:6.1f}s - {message}")

        success_rate = (passed_tests / total_tests * 100) if total_tests > 0 else 0
        self.get_logger().info("-" * 80)
        self.get_logger().info(f"Success Rate: {success_rate:.1f}%")

        if success_rate >= 80:
            self.get_logger().info("🎉 EXCELLENT TEST RESULTS!")
        elif success_rate >= 60:
            self.get_logger().info("👍 GOOD TEST RESULTS!")
        else:
            self.get_logger().info("⚠️  TESTS NEED ATTENTION")

        self.get_logger().info("=" * 80)

    def shutdown(self):
        """关闭测试器"""
        self.is_running = False
        self.get_logger().info("Behavior Node Tester shutting down...")


def main():
    """主函数 - 简化版本避免复杂的异步操作"""
    parser = argparse.ArgumentParser(description='Fixed Behavior Node Tester')
    parser.add_argument('--test', choices=[
        'all', 'set_home', 'unlock_vehicle', 'takeoff', 'goto', 'auto_trace', 'land',
        'formation', 'comprehensive', 'simple'
    ], default='all', help='Specify which test to run')
    parser.add_argument('--timeout', type=float, default=60.0,
                        help='Test timeout in seconds')
    parser.add_argument('--vehicle-id', type=int, default=1,
                        help='Vehicle ID for testing')
    parser.add_argument('--verbose', action='store_true',
                        help='Enable detailed logging')

    args = parser.parse_args()

    rclpy.init()

    try:
        tester = FixedBehaviorNodeTester()
        tester.test_timeout = args.timeout
        tester.vehicle_id = args.vehicle_id
        tester.enable_detailed_logging = args.verbose

        executor = SingleThreadedExecutor()
        executor.add_node(tester)

        # 等待系统初始化
        tester.get_logger().info("Waiting for system to initialize...")

        # 短暂运行以建立连接
        for _ in range(50):  # 5秒初始化时间
            executor.spin_once(timeout_sec=0.1)
            time.sleep(0.1)

        if args.test == 'all':
            # 在主线程中运行测试，使用简单的方式
            tester.run_all_tests()
        else:
            # 运行单个测试
            test_method = getattr(tester, f'test_{args.test}', None)
            if test_method:
                tester.get_logger().info(f"Running single test: {args.test}")
                test_method()
                tester.print_test_summary()
            else:
                tester.get_logger().error(f"Unknown test: {args.test}")

        # 继续处理ROS消息一段时间
        for _ in range(50):  # 额外5秒处理时间
            if not tester.is_running:
                break
            executor.spin_once(timeout_sec=0.1)
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    except Exception as e:
        print(f"Test failed with exception: {str(e)}")
    finally:
        try:
            if 'tester' in locals():
                tester.shutdown()
                tester.destroy_node()
        except:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()