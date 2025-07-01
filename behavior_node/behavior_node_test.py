#!/usr/bin/env python3
"""
Enhanced Behavior Node Test Suite for Typical Mission Tasks
增强型行为节点测试套件，用于典型任务测试
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.executors import SingleThreadedExecutor
import json
import time
import argparse
import math
from enum import Enum
from typing import Dict, List, Any, Optional, Callable, Tuple, Union
from dataclasses import dataclass, field

# ROS2 消息类型
from std_msgs.msg import String, Float32, UInt8, Int32, UInt8MultiArray
from geometry_msgs.msg import Point, Polygon, Point32
from sensor_msgs.msg import Joy
from custom_msgs.msg import (
    SimpleVehicle, ObjectComputation, ObjectLocation, OffboardCtrl,
    StatusTask, CommandRequest, CommandResponse, TaskStage, ParamShort,
    BehaviorTreeStatus, BehaviorNodeStatus
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


class BehaviorTreeState(Enum):
    """行为树状态枚举"""
    IDLE = "IDLE"
    RUNNING = "RUNNING"
    SUCCESS = "SUCCESS"
    FAILURE = "FAILURE"


@dataclass
class TestCase:
    """测试用例数据结构"""
    name: str
    description: str
    tree_name: str
    timeout: float = 30.0
    expected_stage: int = -1
    expected_status: MissionStage = MissionStage.COMPLETE
    required_offboard_count: int = 5
    required_mode_changes: List[int] = field(default_factory=list)
    required_services: List[str] = field(default_factory=list)
    success_criteria: Dict[str, Any] = field(default_factory=dict)
    mission_json: Dict[str, Any] = field(default_factory=dict)
    status: TestStatus = TestStatus.PENDING
    start_time: float = 0.0
    end_time: float = 0.0
    error_message: str = ""
    # 新增行为树相关字段
    expected_tree_status: BehaviorTreeState = BehaviorTreeState.SUCCESS
    required_nodes: List[str] = field(default_factory=list)


@dataclass
class VehicleSimState:
    """飞机仿真状态"""
    is_armed: bool = False
    is_locked: bool = True
    flight_mode: FlightMode = FlightMode.MANUAL
    formation_active: bool = False
    mission_active: bool = False
    rtsp_url: str = "rtsp://192.168.1.100:8554/camera"

    def reset(self) -> None:
        """重置状态"""
        self.is_armed = False
        self.is_locked = True
        self.flight_mode = FlightMode.MANUAL
        self.formation_active = False
        self.mission_active = False


class BehaviorNodeTester(Node):
    """behavior_node测试器，支持典型任务测试"""

    def __init__(self) -> None:
        super().__init__('behavior_node_tester')

        # 测试状态
        self.current_test: Optional[TestCase] = None
        self.test_results: Dict[str, TestCase] = {}
        self.vehicle_state: Optional[SimpleVehicle] = None
        self.task_status: Optional[StatusTask] = None
        self.offboard_commands: List[OffboardCtrl] = []
        self.is_running = True

        # 服务仿真状态
        self.vehicle_sim_state = VehicleSimState()
        self.service_call_count: Dict[str, int] = {}
        self.last_commands: Dict[str, Any] = {}

        # 测试监控状态
        self.mode_changes: List[int] = []
        self.service_calls: List[str] = []
        self.test_start_time = 0.0
        self.test_timeout = 60.0

        # 行为树状态监控
        self.behavior_tree_status: Optional[BehaviorTreeStatus] = None
        self.node_status_history: List[Dict[str, str]] = []

        # 配置参数
        self.vehicle_id = 1
        self.enable_detailed_logging = True
        self.success_rate = 0.95
        self.response_delay = 0.1
        self.simulate_failures = False

        # 初始化ROS接口
        self._setup_qos_profiles()
        self._setup_publishers()
        self._setup_subscribers()
        self._setup_services()

        # 启动仿真数据发布
        self._start_simulation()

        self.get_logger().info("Behavior Node Tester initialized")

    def _setup_qos_profiles(self) -> None:
        """设置QoS配置文件"""
        self.qos_mission = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.qos_control = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        self.qos_sensor = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST
        )

    def _setup_publishers(self) -> None:
        """设置发布器"""
        # 任务JSON发布器 - 核心测试接口
        self.stage_json_pub = self.create_publisher(
            String, 'outer/set/stage_info_json', self.qos_mission)
        self.command_pub = self.create_publisher(
            CommandRequest, 'outer/command/request', self.qos_control)
        self.task_stage_pub = self.create_publisher(
            TaskStage, 'outer/set/task_stage', self.qos_mission)

        # 仿真数据发布器
        self.vehicle_state_pub = self.create_publisher(
            SimpleVehicle, 'inner/information/simple_vehicle', self.qos_mission)
        self.object_detection_pub = self.create_publisher(
            ObjectComputation, 'inner/information/object_computation', self.qos_sensor)
        self.joy_pub = self.create_publisher(
            Joy, 'inner/control/joystick', self.qos_sensor)

    def _setup_subscribers(self) -> None:
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

        # 行为树状态订阅
        self.behavior_tree_status_sub = self.create_subscription(
            BehaviorTreeStatus, 'behavior_tree/status',
            self._behavior_tree_status_callback, self.qos_control)

    def _setup_services(self) -> None:
        """设置服务模拟器"""
        # 飞行模式控制服务
        self.flight_mode_service = self.create_service(CommandLong, 'inner/control/set_flymode',
                                                       self._handle_flight_mode)

        # 锁定/解锁控制服务
        self.lock_unlock_service = self.create_service(CommandBool, 'inner/control/lock_unlock',
                                                       self._handle_lock_unlock)

        # 编队切换控制服务
        self.formation_switch_service = self.create_service(CommandInt, 'inner/control/form_switch',
                                                            self._handle_formation_switch)

        # RTSP URL获取服务
        self.rtsp_url_service = self.create_service(CommandString, 'inner/get/rtsp_url', self._handle_rtsp_url)

    def _start_simulation(self) -> None:
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

    def _publish_vehicle_state(self) -> None:
        """发布模拟的飞机状态"""
        self.sim_time += 0.05

        # 平滑移动到目标位置
        alpha = 0.05
        for key in ['x', 'y', 'z', 'yaw']:
            diff = self.target_position[key] - self.sim_position[key]
            if key == 'yaw':
                if diff > math.pi:
                    diff -= 2 * math.pi
                elif diff < -math.pi:
                    diff += 2 * math.pi
            self.sim_position[key] += diff * alpha

        msg = SimpleVehicle()
        msg.id = self.vehicle_id
        msg.x = int(self.sim_position['x'] * 1000)
        msg.y = int(self.sim_position['y'] * 1000)
        msg.z = int(self.sim_position['z'] * 1000)
        msg.yaw = int(self.sim_position['yaw'] * 1000)

        # 计算速度
        msg.vx = int((self.target_position['x'] - self.sim_position['x']) * 200)
        msg.vy = int((self.target_position['y'] - self.sim_position['y']) * 200)
        msg.vz = int((self.target_position['z'] - self.sim_position['z']) * 200)

        msg.lock = 0 if self.sim_locked else 1
        msg.flymd = self.sim_flight_mode

        # GPS坐标模拟（北京坐标系）
        msg.lat = int((39.9042 + self.sim_position['x'] / 111320.0) * 1e7)
        msg.lon = int((116.4074 + self.sim_position['y'] / 111320.0) * 1e7)
        msg.alt = int(-self.sim_position['z'] * 1000)

        # 其他状态
        msg.pitch = int(math.sin(self.sim_time * 0.1) * 50)
        msg.roll = int(math.cos(self.sim_time * 0.1) * 30)
        msg.type = VehicleType.COPER.value

        self.vehicle_state_pub.publish(msg)
        self.vehicle_state = msg

    def _publish_object_detection(self) -> None:
        """发布模拟的目标检测数据"""
        msg = ObjectComputation()

        # 添加测试目标（模拟动态目标）
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

    def _publish_joy_input(self) -> None:
        """发布模拟的摇杆输入"""
        msg = Joy()
        msg.axes = [0.0] * 8
        msg.buttons = [0] * 11

        if self.current_test and "joystick" in self.current_test.name.lower():
            msg.buttons[0] = 1

        self.joy_pub.publish(msg)

    # =================== 服务处理方法 ===================

    def _simulate_delay(self) -> None:
        """模拟服务响应延迟"""
        if self.response_delay > 0:
            time.sleep(self.response_delay)

    def _should_succeed(self, service_name: str) -> bool:
        """判断服务是否应该成功"""
        if not self.simulate_failures:
            return True
        import random
        return random.random() < self.success_rate

    def _increment_call_count(self, service_name: str) -> None:
        """增加服务调用计数"""
        self.service_call_count[service_name] = self.service_call_count.get(service_name, 0) + 1
        self.service_calls.append(service_name)

    def _handle_flight_mode(self, request: CommandLong.Request, response: CommandLong.Response) -> CommandLong.Response:
        """处理飞行模式控制请求"""
        service_name = "set_flymode"
        self._increment_call_count(service_name)
        self._simulate_delay()

        try:
            flight_mode = FlightMode(request.command)
            takeoff_altitude = request.param7

            self.last_commands[service_name] = {
                'mode': flight_mode.name,
                'command': request.command,
                'param7': takeoff_altitude,
                'timestamp': time.time()
            }

            self.mode_changes.append(request.command)

            if flight_mode == FlightMode.TAKEOFF and self.vehicle_sim_state.is_locked:
                response.success = False
                response.result = 1
                self.get_logger().warn(f"Takeoff rejected: Vehicle locked")
                return response

            if self._should_succeed(service_name):
                self.vehicle_sim_state.flight_mode = flight_mode
                self.sim_flight_mode = flight_mode.value
                response.success = True
                response.result = 0
                self.get_logger().info(f"Flight mode changed to {flight_mode.name}")
                if takeoff_altitude != 0:
                    self.get_logger().info(f"Takeoff altitude: {takeoff_altitude}m")
            else:
                response.success = False
                response.result = 5
                self.get_logger().warn(f"Flight mode change failed (simulated)")

        except ValueError:
            response.success = False
            response.result = 6
            self.get_logger().error(f"Invalid flight mode command: {request.command}")

        return response

    def _handle_lock_unlock(self, request: CommandBool.Request, response: CommandBool.Response) -> CommandBool.Response:
        """处理锁定/解锁控制请求"""
        service_name = "lock_unlock"
        self._increment_call_count(service_name)
        self._simulate_delay()

        unlock_requested = request.value
        action = "unlock" if unlock_requested else "lock"

        self.last_commands[service_name] = {
            'action': action,
            'value': unlock_requested,
            'timestamp': time.time()
        }

        if self._should_succeed(service_name):
            self.vehicle_sim_state.is_locked = not unlock_requested
            self.sim_locked = not unlock_requested
            response.success = True
            response.result = True
            self.get_logger().info(f"Vehicle {action}ed successfully")
        else:
            response.success = False
            response.result = False
            self.get_logger().warn(f"Failed to {action} vehicle (simulated)")

        return response

    def _handle_formation_switch(self, request: CommandInt.Request,
                                 response: CommandInt.Response) -> CommandInt.Response:
        """处理编队切换控制请求"""
        service_name = "form_switch"
        self._increment_call_count(service_name)
        self._simulate_delay()

        frame = request.frame
        command = request.command

        try:
            nav_command_names = {0: "START", 1: "PAUSE", 2: "RESUME", 3: "STOP"}
            nav_command = nav_command_names.get(command, "UNKNOWN")

            self.last_commands[service_name] = {
                'frame': frame,
                'command': nav_command,
                'command_value': command,
                'timestamp': time.time()
            }

            if self._should_succeed(service_name):
                if command == 0:  # START
                    self.vehicle_sim_state.formation_active = True
                elif command == 3:  # STOP
                    self.vehicle_sim_state.formation_active = False

                response.success = True
                frame_desc = "自主控制" if frame == 1 else "仅计算"
                self.get_logger().info(f"Formation {nav_command} executed (frame={frame}: {frame_desc})")
            else:
                response.success = False
                self.get_logger().warn(f"Formation command failed (simulated)")

        except Exception as e:
            response.success = False
            self.get_logger().error(f"Formation command error: {e}")

        return response

    def _handle_rtsp_url(self, request: CommandString.Request,
                         response: CommandString.Response) -> CommandString.Response:
        """处理RTSP URL获取请求"""
        service_name = "rtsp_url"
        self._increment_call_count(service_name)
        self._simulate_delay()

        if self._should_succeed(service_name):
            response.success = True
            response.rslt = self.vehicle_sim_state.rtsp_url
        else:
            response.success = False
            response.rslt = "Failed to get RTSP URL"

        return response

    # =================== 回调方法 ===================

    def _status_callback(self, msg: StatusTask) -> None:
        """任务状态回调"""
        self.task_status = msg

        if self.enable_detailed_logging:
            self.get_logger().info(f"Task status: stage={msg.stage}, id={msg.id}, status={msg.status}")

        if self.current_test:
            self._check_test_completion()

    def _response_callback(self, msg: CommandResponse) -> None:
        """命令响应回调"""
        if self.enable_detailed_logging:
            self.get_logger().info(f"Command response: type={msg.type}, status={msg.status}")

    def _offboard_callback(self, msg: OffboardCtrl) -> None:
        """外部控制指令回调"""
        self.offboard_commands.append(msg)

        if self.enable_detailed_logging:
            self.get_logger().info(f"Offboard control: x={msg.x:.2f}, y={msg.y:.2f}, z={msg.z:.2f}")

        # 更新目标位置以模拟飞行
        self.target_position['x'] = msg.x
        self.target_position['y'] = msg.y
        self.target_position['z'] = msg.z
        self.target_position['yaw'] = msg.yaw

        # 限制命令历史
        if len(self.offboard_commands) > 100:
            self.offboard_commands = self.offboard_commands[-50:]

    def _behavior_tree_status_callback(self, msg: BehaviorTreeStatus) -> None:
        """行为树状态回调"""
        self.behavior_tree_status = msg

        # 记录节点状态历史
        node_status = {}
        for node in msg.nodes:
            node_status[node.name] = node.status
        self.node_status_history.append(node_status)

        if self.enable_detailed_logging:
            self.get_logger().info(f"Behavior tree '{msg.tree_name}' status: {msg.tree_status}")

        if self.current_test:
            self._check_test_completion()

    # =================== 测试逻辑 ===================

    def _check_test_completion(self) -> None:
        """检查测试完成状态"""
        if not self.current_test:
            return

        current_time = time.time()
        test_duration = current_time - self.test_start_time

        # 检查超时
        if test_duration > self.current_test.timeout:
            self._complete_test(TestStatus.TIMEOUT, f"Test timeout after {test_duration:.1f}s")
            return

        # 检查成功条件
        success, message = self._evaluate_test_success()
        if success:
            self._complete_test(TestStatus.PASSED, message)
            return

        # 检查失败条件
        failure, failure_message = self._evaluate_test_failure()
        if failure:
            self._complete_test(TestStatus.FAILED, failure_message)
            return

    def _evaluate_test_success(self) -> Tuple[bool, str]:
        """评估测试成功条件"""
        if not self.current_test:
            return False, "No current test"

        test = self.current_test
        current_time = time.time()
        test_duration = current_time - self.test_start_time

        # 基本运行时间要求
        min_duration = test.success_criteria.get('min_duration', 3.0)
        if test_duration < min_duration:
            return False, f"Test duration {test_duration:.1f}s < minimum {min_duration}s"

        # 检查行为树状态
        if self.behavior_tree_status:
            if test.tree_name and self.behavior_tree_status.tree_name == test.tree_name:
                if self.behavior_tree_status.tree_status == test.expected_tree_status.value:
                    return True, f"Behavior tree '{test.tree_name}' completed with expected status: {test.expected_tree_status.value}"
                elif self.behavior_tree_status.tree_status == "FAILURE":
                    return False, f"Behavior tree '{test.tree_name}' failed"

        # 检查必需的节点是否执行过
        if test.required_nodes:
            executed_nodes = set()
            for node_status in self.node_status_history:
                executed_nodes.update(node_status.keys())

            for required_node in test.required_nodes:
                if required_node not in executed_nodes:
                    return False, f"Required node '{required_node}' not executed"

        # 检查服务调用
        for required_service in test.required_services:
            if required_service not in self.service_calls:
                return False, f"Required service call {required_service} not detected"

        # 检查offboard命令数量
        required_offboard = test.success_criteria.get('required_offboard_count', test.required_offboard_count)
        if len(self.offboard_commands) < required_offboard:
            return False, f"Offboard commands {len(self.offboard_commands)} < required {required_offboard}"

        return True, f"All success criteria met after {test_duration:.1f}s"

    def _evaluate_test_failure(self) -> Tuple[bool, str]:
        """评估测试失败条件"""
        if not self.current_test:
            return False, "No current test"

        # 检查行为树是否失败
        if self.behavior_tree_status and self.behavior_tree_status.tree_status == "FAILURE":
            return True, f"Behavior tree '{self.behavior_tree_status.tree_name}' failed"

        return False, "No failure conditions detected"

    def _complete_test(self, status: TestStatus, message: str) -> None:
        """完成测试"""
        if not self.current_test:
            return

        test = self.current_test
        test.status = status
        test.end_time = time.time()
        test.error_message = message

        duration = test.end_time - test.start_time
        status_str = status.value.upper()

        self.get_logger().info(f"Test '{test.name}' {status_str}: {message} (Duration: {duration:.1f}s)")

        self.test_results[test.name] = test
        self.current_test = None

    # =================== 典型任务测试用例 ===================

    def create_typical_mission_test_cases(self) -> List[TestCase]:
        """创建典型任务测试用例"""
        return [
            # 1. 参数配置任务
            TestCase(
                name="ParameterConfiguration",
                description="测试参数配置任务（SetLine行为树）",
                tree_name="SetLine",
                timeout=15.0,
                expected_tree_status=BehaviorTreeState.SUCCESS,
                required_nodes=["SetLineParameters"],
                success_criteria={'min_duration': 2.0, 'required_offboard_count': 0},
                mission_json={
                    "stage": [{
                        "name": "参数配置",
                        "sn": 1,
                        "cmd": "start",
                        "actions": [{
                            "name": "SetLine",
                            "id": 1,
                            "groupid": 1,
                            "params": [
                                {"name": "vehiType", "type": "string", "value": "多旋翼"},
                                {"name": "spd", "type": "float", "value": 8.0},
                                {"name": "arvDis", "type": "float", "value": 2.0},
                                {"name": "loops", "type": "int", "value": 1},
                                {"name": "pointTag", "type": "string", "value": "loc"},
                                {"name": "wayPoints", "type": "line", "value": [
                                    {"x_lat": 50.0, "y_lon": 30.0, "z_alt": -25.0},
                                    {"x_lat": 100.0, "y_lon": 60.0, "z_alt": -30.0}
                                ]}
                            ],
                            "triggers": [
                                {"name": "triggleType", "type": "string", "value": "manual"}
                            ]
                        }]
                    }]
                }
            ),

            # 2. 载具解锁任务
            TestCase(
                name="VehicleUnlock",
                description="测试载具解锁任务（LockCtrl行为树）",
                tree_name="LockCtrl",
                timeout=15.0,
                expected_tree_status=BehaviorTreeState.SUCCESS,
                required_nodes=["LockControl", "SetLineParameters", "OffBoardControl"],
                required_services=["lock_unlock"],
                success_criteria={'min_duration': 3.0, 'required_offboard_count': 5},
                mission_json={
                    "stage": [{
                        "name": "载具解锁",
                        "sn": 2,
                        "cmd": "start",
                        "actions": [{
                            "name": "LockCtrl",
                            "id": 1,
                            "groupid": 1,
                            "params": [
                                {"name": "vehiType", "type": "string", "value": "多旋翼"},
                                {"name": "spd", "type": "float", "value": 5.0},
                                {"name": "arvDis", "type": "float", "value": 1.0}
                            ],
                            "triggers": [
                                {"name": "triggleType", "type": "string", "value": "manual"}
                            ]
                        }]
                    }]
                }
            ),

            # 3. 起飞任务
            TestCase(
                name="TakeoffMission",
                description="测试起飞任务（TakeOff行为树）",
                tree_name="TakeOff",
                timeout=25.0,
                expected_tree_status=BehaviorTreeState.SUCCESS,
                required_nodes=["LockControl", "FlightModeControl", "SetDestinationPoint", "CheckArriveDestination"],
                required_mode_changes=[FlightMode.TAKEOFF.value],
                required_services=["lock_unlock", "set_flymode"],
                success_criteria={'min_duration': 5.0, 'required_offboard_count': 10},
                mission_json={
                    "stage": [{
                        "name": "起飞",
                        "sn": 3,
                        "cmd": "start",
                        "actions": [{
                            "name": "TakeOff",
                            "id": 1,
                            "groupid": 1,
                            "params": [
                                {"name": "alt", "type": "float", "value": -30.0},
                                {"name": "pointTag", "type": "string", "value": "loc"},
                                {"name": "vehiType", "type": "string", "value": "多旋翼"},
                                {"name": "spd", "type": "float", "value": 5.0},
                                {"name": "arvDis", "type": "float", "value": 2.0}
                            ],
                            "triggers": [
                                {"name": "triggleType", "type": "string", "value": "manual"}
                            ]
                        }]
                    }]
                }
            ),

            # 4. 目标点飞行任务
            TestCase(
                name="GotoDestination",
                description="测试飞向目标点任务（GoToDst行为树）",
                tree_name="GoToDst",
                timeout=30.0,
                expected_tree_status=BehaviorTreeState.SUCCESS,
                required_nodes=["LockControl", "SetDestinationPoint", "CheckArriveDestination"],
                required_services=["lock_unlock"],
                success_criteria={'min_duration': 8.0, 'required_offboard_count': 15},
                mission_json={
                    "stage": [{
                        "name": "飞向目标点",
                        "sn": 4,
                        "cmd": "start",
                        "actions": [{
                            "name": "GoToDst",
                            "id": 1,
                            "groupid": 1,
                            "params": [
                                {"name": "dstLoc", "type": "line", "value": [
                                    {"x_lat": 100.0, "y_lon": 50.0, "z_alt": -25.0}
                                ]},
                                {"name": "pointTag", "type": "string", "value": "loc"},
                                {"name": "spd", "type": "float", "value": 8.0},
                                {"name": "arvDis", "type": "float", "value": 3.0},
                                {"name": "vehiType", "type": "string", "value": "多旋翼"}
                            ],
                            "triggers": [
                                {"name": "triggleType", "type": "string", "value": "manual"}
                            ]
                        }]
                    }]
                }
            ),

            # 5. 自动跟踪任务
            TestCase(
                name="AutoTracking",
                description="测试自动跟踪任务（AutoTrace行为树）",
                tree_name="AutoTrace",
                timeout=20.0,
                expected_tree_status=BehaviorTreeState.SUCCESS,
                required_nodes=["LockControl", "CheckQuitSearch", "SetDestinationPoint"],
                required_services=["lock_unlock"],
                success_criteria={'min_duration': 6.0, 'required_offboard_count': 8},
                mission_json={
                    "stage": [{
                        "name": "自动跟踪",
                        "sn": 5,
                        "cmd": "start",
                        "actions": [{
                            "name": "AutoTrace",
                            "id": 1,
                            "groupid": 1,
                            "params": [
                                {"name": "vehiType", "type": "string", "value": "多旋翼"},
                                {"name": "spd", "type": "float", "value": 6.0},
                                {"name": "arvDis", "type": "float", "value": 2.0}
                            ],
                            "triggers": [
                                {"name": "triggleType", "type": "string", "value": "manual"}
                            ]
                        }]
                    }]
                }
            ),

            # 6. 降落任务
            TestCase(
                name="LandingMission",
                description="测试降落任务（Land行为树）",
                tree_name="Land",
                timeout=20.0,
                expected_tree_status=BehaviorTreeState.SUCCESS,
                required_nodes=["FlightModeControl", "SetDestinationPoint", "OffBoardControl"],
                required_mode_changes=[FlightMode.LAND.value],
                success_criteria={'min_duration': 4.0, 'required_offboard_count': 8},
                mission_json={
                    "stage": [{
                        "name": "降落",
                        "sn": 6,
                        "cmd": "start",
                        "actions": [{
                            "name": "Land",
                            "id": 1,
                            "groupid": 1,
                            "params": [
                                {"name": "vehiType", "type": "string", "value": "多旋翼"}
                            ],
                            "triggers": [
                                {"name": "triggleType", "type": "string", "value": "manual"}
                            ]
                        }]
                    }]
                }
            ),

            # 7. 编队飞行任务
            TestCase(
                name="FormationFlight",
                description="测试编队飞行任务（FormFly行为树）",
                tree_name="FormFly",
                timeout=25.0,
                expected_tree_status=BehaviorTreeState.SUCCESS,
                required_nodes=["LockControl", "NavigationControl", "SetDestinationPoint"],
                required_services=["lock_unlock", "form_switch"],
                success_criteria={'min_duration': 8.0, 'required_offboard_count': 12},
                mission_json={
                    "stage": [{
                        "name": "编队飞行",
                        "sn": 7,
                        "cmd": "start",
                        "actions": [{
                            "name": "FormFly",
                            "id": 1,
                            "groupid": 130,
                            "params": [
                                {"name": "wayPoints", "type": "line", "value": [
                                    {"x_lat": 23.661289788690084, "y_lon": 107.02720853967423, "z_alt": 30.0},
                                    {"x_lat": 23.659265430554882, "y_lon": 107.03040573281862, "z_alt": 30.0}
                                ]},
                                {"name": "pointTag", "type": "string", "value": "gps"},
                                {"name": "vehiType", "type": "string", "value": "多旋翼"},
                                {"name": "formOffset", "type": "point", "value": {
                                    "diff_x_lat": 0.0,
                                    "diff_y_lon": 0.0,
                                    "diff_z_alt": 0.0
                                }},
                                {"name": "spd", "type": "float", "value": 10.0},
                                {"name": "loops", "type": "int", "value": 1},
                                {"name": "arvDis", "type": "float", "value": 1.0}
                            ],
                            "triggers": [
                                {"name": "triggleType", "type": "string", "value": "manual"}
                            ]
                        }]
                    }]
                }
            ),

            # 8. 搜索任务
            TestCase(
                name="SearchMission",
                description="测试搜索任务（Search行为树）",
                tree_name="PatternSearch",
                timeout=30.0,
                expected_tree_status=BehaviorTreeState.SUCCESS,
                required_nodes=["LockControl", "CheckQuitSearch", "SetDestinationPoint"],
                required_services=["lock_unlock"],
                success_criteria={'min_duration': 10.0, 'required_offboard_count': 15},
                mission_json={
                    "stage": [{
                        "name": "区域搜索",
                        "sn": 8,
                        "cmd": "start",
                        "actions": [{
                            "name": "PatternSearch",
                            "id": 1,
                            "groupid": 1,
                            "params": [
                                {"name": "areaPoints", "type": "line", "value": [
                                    {"x_lat": 50.0, "y_lon": 50.0, "z_alt": -30.0},
                                    {"x_lat": 150.0, "y_lon": 50.0, "z_alt": -30.0},
                                    {"x_lat": 150.0, "y_lon": 150.0, "z_alt": -30.0},
                                    {"x_lat": 50.0, "y_lon": 150.0, "z_alt": -30.0}
                                ]},
                                {"name": "pointTag", "type": "string", "value": "loc"},
                                {"name": "vehiType", "type": "string", "value": "多旋翼"},
                                {"name": "spd", "type": "float", "value": 8.0},
                                {"name": "arvDis", "type": "float", "value": 8.0}
                            ],
                            "triggers": [
                                {"name": "triggleType", "type": "string", "value": "manual"}
                            ]
                        }]
                    }]
                }
            ),

            # 9. 悬停任务
            TestCase(
                name="HoverMission",
                description="测试悬停任务（Hover行为树）",
                tree_name="Hover",
                timeout=35.0,
                expected_tree_status=BehaviorTreeState.SUCCESS,
                required_nodes=["LockControl", "SetDestinationPoint", "OffBoardControl"],
                required_services=["lock_unlock"],
                success_criteria={'min_duration': 5.0, 'required_offboard_count': 10},
                mission_json={
                    "stage": [{
                        "name": "悬停",
                        "sn": 9,
                        "cmd": "start",
                        "actions": [{
                            "name": "Hover",
                            "id": 1,
                            "groupid": 1,
                            "params": [
                                {"name": "vehiType", "type": "string", "value": "多旋翼"},
                                {"name": "spd", "type": "float", "value": 0.0}
                            ],
                            "triggers": [
                                {"name": "triggleType", "type": "string", "value": "manual"}
                            ]
                        }]
                    }]
                }
            ),

            # 10. 综合任务
            TestCase(
                name="ComprehensiveMission",
                description="测试综合任务（Comprehensive行为树）",
                tree_name="ComprehensiveMission",
                timeout=45.0,
                expected_tree_status=BehaviorTreeState.SUCCESS,
                required_nodes=["LockControl", "FlightModeControl", "SetDestinationPoint", "CheckQuitSearch"],
                required_mode_changes=[FlightMode.TAKEOFF.value, FlightMode.LAND.value],
                required_services=["lock_unlock", "set_flymode"],
                success_criteria={'min_duration': 20.0, 'required_offboard_count': 30},
                mission_json={
                    "stage": [{
                        "name": "综合任务",
                        "sn": 10,
                        "cmd": "start",
                        "actions": [{
                            "name": "ComprehensiveMission",
                            "id": 1,
                            "groupid": 1,
                            "params": [
                                {"name": "vehiType", "type": "string", "value": "多旋翼"},
                                {"name": "spd", "type": "float", "value": 8.0},
                                {"name": "arvDis", "type": "float", "value": 3.0},
                                {"name": "pointTag", "type": "string", "value": "loc"},
                                {"name": "dstLoc", "type": "line", "value": [
                                    {"x_lat": 100.0, "y_lon": 80.0, "z_alt": -25.0}
                                ]}
                            ],
                            "triggers": [
                                {"name": "triggleType", "type": "string", "value": "manual"}
                            ]
                        }]
                    }]
                }
            )
        ]

    def send_mission_json(self, test_case: TestCase) -> None:
        """发送任务JSON"""
        msg = String()
        msg.data = json.dumps(test_case.mission_json)
        self.stage_json_pub.publish(msg)

        self.get_logger().info(f"Sent mission JSON for {test_case.name}")
        if self.enable_detailed_logging:
            self.get_logger().info(f"JSON content: {json.dumps(test_case.mission_json, indent=2)}")

    def run_test_case(self, test_case: TestCase) -> None:
        """运行单个测试用例"""
        self.reset_test_state()
        self.current_test = test_case
        self.test_start_time = time.time()
        test_case.start_time = self.test_start_time
        test_case.status = TestStatus.RUNNING

        self.get_logger().info(f"Starting typical mission test: {test_case.name}")
        self.get_logger().info(f"Description: {test_case.description}")

        try:
            # 发送任务JSON
            self.send_mission_json(test_case)

            # 等待测试完成
            self.wait_for_test_completion(test_case.timeout)

        except Exception as e:
            if self.current_test:
                self._complete_test(TestStatus.FAILED, f"Test execution error: {str(e)}")

    def wait_for_test_completion(self, timeout: float = 30.0) -> None:
        """等待测试完成"""
        start_time = time.time()

        while self.current_test and (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)

            if self.current_test:
                self._check_test_completion()

            time.sleep(0.01)

    def reset_test_state(self) -> None:
        """重置测试状态"""
        self.offboard_commands.clear()
        self.mode_changes.clear()
        self.service_calls.clear()
        self.service_call_count.clear()
        self.last_commands.clear()
        self.vehicle_sim_state.reset()
        self.behavior_tree_status = None
        self.node_status_history.clear()

    def run_all_typical_mission_tests(self) -> None:
        """运行所有典型任务测试"""
        self.get_logger().info("=" * 80)
        self.get_logger().info("STARTING TYPICAL MISSION TESTS FOR BEHAVIOR NODE")
        self.get_logger().info("=" * 80)

        test_cases = self.create_typical_mission_test_cases()

        for i, test_case in enumerate(test_cases, 1):
            if not self.is_running:
                break

            self.get_logger().info(f"\n[{i}/{len(test_cases)}] Running typical mission test: {test_case.name}")
            self.run_test_case(test_case)

            # 测试间隔期间处理ROS消息
            for _ in range(30):  # 3秒间隔
                rclpy.spin_once(self, timeout_sec=0.1)

        self.print_test_summary()

    def print_test_summary(self) -> None:
        """打印测试总结"""
        self.get_logger().info("\n" + "=" * 80)
        self.get_logger().info("TYPICAL MISSION TEST SUMMARY")
        self.get_logger().info("=" * 80)

        total_tests = len(self.test_results)
        passed_tests = sum(1 for test in self.test_results.values() if test.status == TestStatus.PASSED)
        failed_tests = sum(1 for test in self.test_results.values() if test.status == TestStatus.FAILED)
        timeout_tests = sum(1 for test in self.test_results.values() if test.status == TestStatus.TIMEOUT)

        self.get_logger().info(f"Total Tests:    {total_tests}")
        self.get_logger().info(f"Passed:         {passed_tests}")
        self.get_logger().info(f"Failed:         {failed_tests}")
        self.get_logger().info(f"Timeout:        {timeout_tests}")
        self.get_logger().info("-" * 80)

        for test_name, test in self.test_results.items():
            status_str = test.status.value.upper()
            duration = test.end_time - test.start_time if test.end_time > 0 else 0
            message = test.error_message or "OK"

            if test.status == TestStatus.PASSED:
                symbol = "✓"
            elif test.status == TestStatus.FAILED:
                symbol = "✗"
            elif test.status == TestStatus.TIMEOUT:
                symbol = "⏰"
            else:
                symbol = "?"

            self.get_logger().info(f"{symbol} {test_name:25} [{status_str:7}] {duration:6.1f}s - {message}")

        success_rate = (passed_tests / total_tests * 100) if total_tests > 0 else 0
        self.get_logger().info("-" * 80)
        self.get_logger().info(f"Success Rate: {success_rate:.1f}%")

        if success_rate >= 80:
            self.get_logger().info("🎉 EXCELLENT TYPICAL MISSION TEST RESULTS!")
        elif success_rate >= 60:
            self.get_logger().info("👍 GOOD TYPICAL MISSION TEST RESULTS!")
        else:
            self.get_logger().info("⚠️  TYPICAL MISSION TESTS NEED ATTENTION")

        # 打印服务调用统计
        self.get_logger().info("\nService Call Statistics:")
        for service, count in self.service_call_count.items():
            self.get_logger().info(f"  {service}: {count} calls")

        # 打印行为树状态统计
        if self.behavior_tree_status:
            self.get_logger().info(f"\nLast Behavior Tree Status:")
            self.get_logger().info(f"  Tree: {self.behavior_tree_status.tree_name}")
            self.get_logger().info(f"  Status: {self.behavior_tree_status.tree_status}")
            self.get_logger().info(f"  Node count: {len(self.behavior_tree_status.nodes)}")

        self.get_logger().info("=" * 80)

    def shutdown(self) -> None:
        """关闭测试器"""
        self.is_running = False
        self.get_logger().info("Enhanced Behavior Node Tester shutting down...")


def main() -> None:
    """主函数"""
    parser = argparse.ArgumentParser(description='Enhanced Behavior Node Tester for Typical Missions')
    parser.add_argument('--test', choices=[
        'all', 'parameter_config', 'vehicle_unlock', 'takeoff', 'goto_destination',
        'auto_tracking', 'landing', 'formation_flight', 'search', 'hover', 'comprehensive'
    ], default='all', help='Specify which typical mission test to run')
    parser.add_argument('--timeout', type=float, default=60.0, help='Test timeout in seconds')
    parser.add_argument('--vehicle-id', type=int, default=1, help='Vehicle ID for testing')
    parser.add_argument('--verbose', action='store_true', help='Enable detailed logging')
    parser.add_argument('--simulate-failures', action='store_true', help='Enable failure simulation')
    parser.add_argument('--success-rate', type=float, default=0.95, help='Service success rate')

    args = parser.parse_args()

    rclpy.init()

    try:
        tester = BehaviorNodeTester()
        tester.test_timeout = args.timeout
        tester.vehicle_id = args.vehicle_id
        tester.enable_detailed_logging = args.verbose
        tester.simulate_failures = args.simulate_failures
        tester.success_rate = args.success_rate

        executor = SingleThreadedExecutor()
        executor.add_node(tester)

        # 等待系统初始化
        tester.get_logger().info("Waiting for system to initialize...")
        for _ in range(50):
            executor.spin_once(timeout_sec=0.1)
            time.sleep(0.1)

        if args.test == 'all':
            tester.run_all_typical_mission_tests()
        else:
            # 运行单个测试
            test_cases = tester.create_typical_mission_test_cases()
            target_test = None
            for test_case in test_cases:
                test_name_simplified = test_case.name.lower().replace('mission', '').replace('tracking', '').replace(
                    'configuration', 'config')
                if args.test.replace('_', '') in test_name_simplified:
                    target_test = test_case
                    break

            if target_test:
                tester.get_logger().info(f"Running single typical mission test: {target_test.name}")
                tester.run_test_case(target_test)
                tester.print_test_summary()
            else:
                tester.get_logger().error(f"Unknown test: {args.test}")

        # 继续处理ROS消息一段时间
        for _ in range(50):
            if not tester.is_running:
                break
            executor.spin_once(timeout_sec=0.1)
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nTypical mission test interrupted by user")
    except Exception as e:
        print(f"Typical mission test failed with exception: {str(e)}")
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
