#!/usr/bin/env python3
"""
Behavior Node Test Suite for Typical Mission Tasks
行为节点测试套件，用于典型任务测试
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
    parameters: Dict[str, Any] = field(default_factory=dict)
    status: TestStatus = TestStatus.PENDING
    start_time: float = 0.0
    end_time: float = 0.0
    error_message: str = ""
    setup_params: Dict[str, Any] = field(default_factory=dict)
    expected_tree_status: BehaviorTreeState = BehaviorTreeState.SUCCESS
    required_nodes: List[str] = field(default_factory=list)

    test_category: str = "basic"  # basic, advanced, error_handling, integration
    prerequisites: List[str] = field(default_factory=list)  # 前置条件测试
    cleanup_required: bool = True  # 是否需要清理


@dataclass
class VehicleSimState:
    """飞机仿真状态"""
    is_armed: bool = False
    is_locked: bool = True
    flight_mode: FlightMode = FlightMode.MANUAL
    formation_active: bool = False
    mission_active: bool = False
    trace_attack_active: bool = False
    joystick_control_active: bool = False
    rtsp_url: str = "rtsp://192.168.1.100:8554/camera"
    error_simulation_enabled: bool = False

    def reset(self) -> None:
        """重置状态"""
        self.is_armed = False
        self.is_locked = True
        self.flight_mode = FlightMode.MANUAL
        self.formation_active = False
        self.mission_active = False
        self.trace_attack_active = False
        self.joystick_control_active = False
        self.error_simulation_enabled = False


class BehaviorNodeTester(Node):
    """behavior_node测试器，支持典型任务测试"""

    def __init__(self) -> None:
        super().__init__('behavior_node_tester')

        # 测试状态
        self.current_test: Optional[TestCase] = None
        self.test_results: Dict[str, TestCase] = {}
        self.test_statistics = {
            'total': 0, 'passed': 0, 'failed': 0, 'timeout': 0, 'skipped': 0,
            'basic_tests': 0, 'advanced_tests': 0, 'error_tests': 0, 'integration_tests': 0
        }

        # 系统状态
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
        self.node_execution_count: Dict[str, int] = {}

        # 配置参数
        self.vehicle_id = 1
        self.enable_detailed_logging = True
        self.success_rate = 0.95
        self.response_delay = 0.1
        self.simulate_failures = False
        self.test_parallel_execution = False

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
        # 任务控制发布器
        self.mission_json_pub = self.create_publisher(
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
        self.flight_mode_service = self.create_service(
            CommandLong, 'inner/control/set_flymode', self._handle_flight_mode)

        # 锁定/解锁控制服务
        self.lock_unlock_service = self.create_service(
            CommandBool, 'inner/control/lock_unlock', self._handle_lock_unlock)

        # 编队切换控制服务
        self.formation_switch_service = self.create_service(
            CommandInt, 'inner/control/form_switch', self._handle_formation_switch)

        # 跟踪攻击控制服务
        self.guidance_switch_service = self.create_service(
            CommandInt, 'inner/control/guidance_switch', self._handle_guidance_switch)

        # RTSP URL获取服务
        self.rtsp_url_service = self.create_service(
            CommandString, 'inner/get/rtsp_url', self._handle_rtsp_url)

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
        alpha = 0.3
        min_movement = 0.1  # 最小移动距离（米）

        for key in ['x', 'y', 'z', 'yaw']:
            diff = self.target_position[key] - self.sim_position[key]
            if key == 'yaw':
                if diff > math.pi:
                    diff -= 2 * math.pi
                elif diff < -math.pi:
                    diff += 2 * math.pi

            # 如果距离很小，直接到达目标位置
            if abs(diff) < min_movement and key != 'yaw':
                self.sim_position[key] = self.target_position[key]
            else:
                self.sim_position[key] += diff * alpha

        # 确保单位一致性 - 关键修复
        msg = SimpleVehicle()
        msg.id = self.vehicle_id
        msg.x = int(self.sim_position['x'] * 1000)  # 转换为毫米
        msg.y = int(self.sim_position['y'] * 1000)  # 转换为毫米
        msg.z = int(self.sim_position['z'] * 1000)  # 转换为毫米
        msg.yaw = int(self.sim_position['yaw'] * 1000)

        # 计算合理的速度
        msg.vx = int((self.target_position['x'] - self.sim_position['x']) * 1000)
        msg.vy = int((self.target_position['y'] - self.sim_position['y']) * 1000)
        msg.vz = int((self.target_position['z'] - self.sim_position['z']) * 1000)

        msg.lock = 0 if self.sim_locked else 1
        msg.flymd = self.sim_flight_mode

        # GPS坐标模拟
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

        # 改进名称匹配逻辑
        if self.current_test:
            test_name_lower = self.current_test.name.lower()
            tree_name_lower = self.current_test.tree_name.lower() if self.current_test.tree_name else ""

            # 检查是否需要目标检测数据的测试
            needs_objects = any(keyword in test_name_lower or keyword in tree_name_lower
                                for keyword in ["trace", "attack", "search", "auto"])

            if needs_objects:
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
        msg.buttons = [1500, 1500, 1500, 1500] + [0] * 7

        # 在摇杆控制测试时模拟输入
        if self.current_test and "joystick" in self.current_test.name.lower():
            msg.buttons[0] = 1600 + int(math.sin(self.sim_time) * 100)  # X轴
            msg.buttons[1] = 1500 + int(math.cos(self.sim_time) * 100)  # Y轴
            msg.buttons[2] = 1550  # 油门
            msg.buttons[3] = 1500  # 航向

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

        # 错误处理测试时的特殊逻辑
        if self.current_test and self.current_test.test_category == "error_handling":
            if "error" in self.current_test.name.lower():
                return False

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

        return response

    def _handle_formation_switch(self, request: CommandInt.Request,
                                 response: CommandInt.Response) -> CommandInt.Response:
        """处理编队切换控制请求"""
        service_name = "form_switch"
        self._increment_call_count(service_name)
        self._simulate_delay()

        frame = request.frame
        command = request.command

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
            self.get_logger().info(f"Formation {nav_command} executed")
        else:
            response.success = False

        return response

    def _handle_guidance_switch(self, request: CommandInt.Request,
                                response: CommandInt.Response) -> CommandInt.Response:
        """处理跟踪攻击控制请求"""
        service_name = "guidance_switch"
        self._increment_call_count(service_name)
        self._simulate_delay()

        frame = request.frame
        command = request.command
        current = request.current

        action_names = {0: "START_TRACE", 1: "STOP_TRACE", 2: "START_ATTACK", 3: "STOP_ATTACK"}
        action = action_names.get(command, "UNKNOWN")

        self.last_commands[service_name] = {
            'frame': frame,
            'command': action,
            'current': current,
            'timestamp': time.time()
        }

        if self._should_succeed(service_name):
            if command in [0, 2]:  # START operations
                self.vehicle_sim_state.trace_attack_active = True
            elif command in [1, 3]:  # STOP operations
                self.vehicle_sim_state.trace_attack_active = False

            response.success = True
            self.get_logger().info(f"Trace/Attack {action} executed")
        else:
            response.success = False

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

    def _response_callback(self, msg: CommandResponse) -> None:
        """命令响应回调"""
        if self.enable_detailed_logging:
            self.get_logger().info(f"Command response: type={msg.type}, status={msg.status}")

    def _offboard_callback(self, msg: OffboardCtrl) -> None:
        """外部控制指令回调"""
        self.offboard_commands.append(msg)

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

        # 记录节点执行次数
        for node in msg.nodes:
            if node.status in ["RUNNING", "SUCCESS"]:
                self.node_execution_count[node.name] = self.node_execution_count.get(node.name, 0) + 1

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

        # 行为树状态检查
        if self.behavior_tree_status:
            if test.tree_name and self.behavior_tree_status.tree_name == test.tree_name:
                if self.behavior_tree_status.tree_status == test.expected_tree_status.value:
                    return True, f"Behavior tree '{test.tree_name}' completed with expected status"

        # 检查必需的节点执行
        if test.required_nodes:
            for required_node in test.required_nodes:
                if required_node not in self.node_execution_count or self.node_execution_count[required_node] < 1:
                    return False, f"Required node '{required_node}' not executed"

        # 原有检查逻辑
        return self._legacy_test_success_check(test, test_duration)

    def _legacy_test_success_check(self, test: TestCase, test_duration: float) -> Tuple[bool, str]:
        """原有的测试成功条件检查"""
        required_offboard = test.success_criteria.get('required_offboard_count', test.required_offboard_count)
        if len(self.offboard_commands) < required_offboard:
            return False, f"Offboard commands {len(self.offboard_commands)} < required {required_offboard}"

        # 检查必需的模式变化
        for required_mode in test.required_mode_changes:
            if required_mode not in self.mode_changes:
                return False, f"Required mode change {required_mode} not detected"

        # 检查必需的服务调用
        for required_service in test.required_services:
            if required_service not in self.service_calls:
                return False, f"Required service call {required_service} not detected"

        # 特定测试的成功条件
        return self._check_specific_test_success(test, test_duration)

    def _check_specific_test_success(self, test: TestCase, test_duration: float) -> Tuple[bool, str]:
        """检查特定测试的成功条件"""
        if test.tree_name == "Emergency":
            if FlightMode.LAND.value not in self.mode_changes:
                return False, "Emergency land mode not activated"
        elif test.tree_name == "TraceAttack":
            # 检查服务是否被调用
            if "guidance_switch" in self.service_calls:
                # 如果检测到目标或者运行足够长时间，认为成功
                if (self.node_execution_count.get("CheckQuitSearch", 0) > 5 or
                        test_duration > 15.0):
                    return True, "TraceAttack test completed successfully"
            return False, "TraceAttack service not called or insufficient execution"
        elif test.tree_name == "JoyStick":
            if not self.vehicle_sim_state.joystick_control_active:
                return False, "JoyStick control not activated"
        elif test.tree_name == "WaypointMission":
            if len(self.offboard_commands) < 20:
                return False, f"Insufficient waypoint loop commands: {len(self.offboard_commands)}"

        return True, f"All success criteria met after {test_duration:.1f}s"

    def _evaluate_test_failure(self) -> Tuple[bool, str]:
        """评估测试失败条件"""
        if not self.current_test:
            return False, "No current test"

        # 检查行为树失败
        if self.behavior_tree_status and self.behavior_tree_status.tree_status == "FAILURE":
            # 对于错误处理测试，失败可能是预期的
            if self.current_test.test_category == "error_handling":
                return False, "Expected failure in error handling test"
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
        self.get_logger().info(f"Test '{test.name}' {status.value.upper()}: {message} (Duration: {duration:.1f}s)")

        # 更新统计
        self.test_statistics[status.value] += 1
        self.test_statistics[f"{test.test_category}_tests"] += 1

        self.test_results[test.name] = test
        self.current_test = None

    # =================== 创建测试用例 ===================

    def create_test_cases(self) -> List[TestCase]:
        """创建所有测试用例"""
        test_cases = []

        # 基础功能测试
        test_cases.extend(self._create_basic_tests())

        # 高级功能测试
        test_cases.extend(self._create_advanced_tests())

        # 错误处理测试
        if self.simulate_failures:
            test_cases.extend(self._create_error_handling_tests())

        # 集成测试
        test_cases.extend(self._create_integration_tests())

        return test_cases

    def _create_basic_tests(self) -> List[TestCase]:
        """创建基础功能测试"""
        return [
            TestCase(
                name="SetHome",
                description="Test setting home point",
                tree_name="",
                timeout=10.0,
                test_category="basic",
                success_criteria={'min_duration': 1.0, 'required_offboard_count': 0}
            ),
            TestCase(
                name="ParameterConfig",
                description="Test parameter configuration using SetLine behavior tree",
                tree_name="SetLine",
                timeout=15.0,
                test_category="basic",
                expected_tree_status=BehaviorTreeState.SUCCESS,
                required_nodes=["SetLineParameters"],
                success_criteria={'min_duration': 2.0, 'required_offboard_count': 0}
            ),
            TestCase(
                name="UnlockVehicle",
                description="Test vehicle unlock using LockCtrl behavior tree",
                tree_name="LockCtrl",
                timeout=15.0,
                test_category="basic",
                expected_tree_status=BehaviorTreeState.SUCCESS,
                required_nodes=["LockControl", "SetLineParameters", "OffBoardControl"],
                required_services=["lock_unlock"],
                success_criteria={'min_duration': 3.0, 'required_offboard_count': 5}
            ),
            TestCase(
                name="TakeOff",
                description="Test takeoff using TakeOff behavior tree",
                tree_name="TakeOff",
                timeout=25.0,
                test_category="basic",
                expected_tree_status=BehaviorTreeState.SUCCESS,
                required_nodes=["LockControl", "FlightModeControl", "SetDestinationPoint"],
                required_mode_changes=[FlightMode.TAKEOFF.value],
                required_services=["lock_unlock", "set_flymode"],
                success_criteria={'min_duration': 5.0, 'required_offboard_count': 10}
            ),
            TestCase(
                name="Hover",
                description="Test hover using Hover behavior tree",
                tree_name="Hover",
                timeout=26.0,
                test_category="basic",
                expected_tree_status=BehaviorTreeState.SUCCESS,
                required_nodes=["LockControl", "SetDestinationPoint", "OffBoardControl"],
                required_services=["lock_unlock"],
                success_criteria={'min_duration': 8.0, 'required_offboard_count': 15}
            ),
            TestCase(
                name="Land",
                description="Test landing using Land behavior tree",
                tree_name="Land",
                timeout=20.0,
                test_category="basic",
                expected_tree_status=BehaviorTreeState.SUCCESS,
                required_nodes=["FlightModeControl", "SetDestinationPoint", "OffBoardControl"],
                required_mode_changes=[FlightMode.LAND.value],
                success_criteria={'min_duration': 4.0, 'required_offboard_count': 8}
            )
        ]

    def _create_advanced_tests(self) -> List[TestCase]:
        """创建高级功能测试"""
        return [
            TestCase(
                name="GotoDestination",
                description="Test goto destination using GoToDst behavior tree",
                tree_name="GoToDst",
                timeout=50.0,
                test_category="advanced",
                expected_tree_status=BehaviorTreeState.SUCCESS,
                required_nodes=["LockControl", "SetDestinationPoint", "CheckArriveDestination"],
                required_services=["lock_unlock"],
                success_criteria={'min_duration': 8.0, 'required_offboard_count': 15}
            ),
            TestCase(
                name="AutoTrace",
                description="Test auto trace using AutoTrace behavior tree",
                tree_name="AutoTrace",
                timeout=25.0,
                test_category="advanced",
                expected_tree_status=BehaviorTreeState.SUCCESS,
                required_nodes=["LockControl", "CheckQuitSearch", "SetDestinationPoint"],
                required_services=["lock_unlock"],
                success_criteria={'min_duration': 8.0, 'required_offboard_count': 10}
            ),
            TestCase(
                name="TraceAttack",
                description="Test trace attack using TraceAttack behavior tree",
                tree_name="TraceAttack",
                timeout=50.0,
                test_category="advanced",
                expected_tree_status=BehaviorTreeState.SUCCESS,
                required_nodes=["LockControl", "TraceAttackControl", "CheckQuitSearch"],
                required_services=["lock_unlock", "guidance_switch"],
                success_criteria={'min_duration': 10.0, 'required_offboard_count': 12}
            ),
            TestCase(
                name="PatternSearch",
                description="Test pattern search using Search behavior tree",
                tree_name="PatternSearch",
                timeout=35.0,
                test_category="advanced",
                expected_tree_status=BehaviorTreeState.SUCCESS,
                required_nodes=["LockControl", "CheckQuitSearch", "SetDestinationPoint"],
                required_services=["lock_unlock"],
                success_criteria={'min_duration': 12.0, 'required_offboard_count': 15}
            ),
            TestCase(
                name="WaypointMission",
                description="Test multi-loop waypoint mission using WaypointMission behavior tree",
                tree_name="WaypointMission",
                timeout=40.0,
                test_category="advanced",
                expected_tree_status=BehaviorTreeState.SUCCESS,
                required_nodes=["LockControl", "CheckQuitLineLoop", "SetDestinationPoint"],
                required_services=["lock_unlock"],
                success_criteria={'min_duration': 15.0, 'required_offboard_count': 20}
            ),
            TestCase(
                name="FormationFlight",
                description="Test formation flight using FormFly behavior tree",
                tree_name="FormFly",
                timeout=35.0,
                test_category="advanced",
                expected_tree_status=BehaviorTreeState.SUCCESS,
                required_nodes=["LockControl", "NavigationControl", "SetDestinationPoint"],
                required_services=["lock_unlock", "form_switch"],
                success_criteria={'min_duration': 12.0, 'required_offboard_count': 15}
            ),
            TestCase(
                name="JoystickControl",
                description="Test joystick control using JoyStick behavior tree",
                tree_name="JoyStick",
                timeout=25.0,
                test_category="advanced",
                expected_tree_status=BehaviorTreeState.SUCCESS,
                required_nodes=["LockControl", "JoyControl", "FlightModeControl"],
                required_services=["lock_unlock", "set_flymode"],
                required_mode_changes=[FlightMode.OFFBOARD.value],
                success_criteria={'min_duration': 8.0, 'required_offboard_count': 0}
            )
        ]

    def _create_error_handling_tests(self) -> List[TestCase]:
        """创建错误处理测试"""
        return [
            TestCase(
                name="ServiceFailureHandling",
                description="Test behavior when services fail",
                tree_name="LockCtrl",
                timeout=20.0,
                test_category="error_handling",
                expected_tree_status=BehaviorTreeState.FAILURE,
                success_criteria={'min_duration': 3.0}
            ),
            TestCase(
                name="TimeoutHandling",
                description="Test behavior tree timeout handling",
                tree_name="TakeOff",
                timeout=5.0,  # 故意设短超时
                test_category="error_handling",
                success_criteria={'min_duration': 4.0}
            )
        ]

    def _create_integration_tests(self) -> List[TestCase]:
        """创建集成测试"""
        return [
            TestCase(
                name="ComprehensiveMission",
                description="Test comprehensive mission using Comprehensive behavior tree",
                tree_name="ComprehensiveMission",
                timeout=60.0,
                test_category="integration",
                expected_tree_status=BehaviorTreeState.SUCCESS,
                required_nodes=["LockControl", "FlightModeControl", "SetDestinationPoint",
                                "CheckArriveDestination", "CheckQuitSearch"],
                required_services=["lock_unlock", "set_flymode"],
                required_mode_changes=[FlightMode.TAKEOFF.value, FlightMode.LAND.value],
                success_criteria={'min_duration': 30.0, 'required_offboard_count': 40}
            ),
            TestCase(
                name="EmergencyLanding",
                description="Test emergency landing using Emergency behavior tree",
                tree_name="Emergency",
                timeout=25.0,
                test_category="integration",
                expected_tree_status=BehaviorTreeState.SUCCESS,
                required_nodes=["FlightModeControl", "SetDestinationPoint", "OffBoardControl"],
                required_mode_changes=[FlightMode.LAND.value],
                success_criteria={'min_duration': 8.0, 'required_offboard_count': 10}
            )
        ]

    # =================== 测试执行方法 ===================

    def run_test_case(self, test_case: TestCase) -> None:
        """运行单个测试用例"""
        self.reset_test_state()
        self.current_test = test_case
        self.test_start_time = time.time()
        test_case.start_time = self.test_start_time
        test_case.status = TestStatus.RUNNING

        self.get_logger().info(f"Starting {test_case.test_category} test: {test_case.name}")

        # 设置错误模拟
        if test_case.test_category == "error_handling":
            self.vehicle_sim_state.error_simulation_enabled = True

        try:
            # 根据测试名称执行对应的测试逻辑
            test_method_name = f"_test_{test_case.name.lower().replace(' ', '_')}"
            test_method = getattr(self, test_method_name, None)

            if test_method:
                test_method()
            else:
                # 通用测试逻辑
                self._execute_generic_test(test_case)

            self.wait_for_test_completion(test_case.timeout)

        except Exception as e:
            if self.current_test:
                self._complete_test(TestStatus.FAILED, f"Test execution error: {str(e)}")
        finally:
            # 清理错误模拟状态
            self.vehicle_sim_state.error_simulation_enabled = False

    def _execute_generic_test(self, test_case: TestCase) -> None:
        """执行通用测试逻辑"""
        if test_case.tree_name:
            params = test_case.parameters or self._get_default_parameters(test_case.tree_name)
            self.send_mission_json(test_case.name, 1, test_case.tree_name, params)

    def _get_default_parameters(self, tree_name: str) -> Dict[str, Any]:
        """获取默认测试参数"""

        # 基础通用参数
        base_params = {
            "vehiType": "多旋翼",
            "spd": 8.0,
            "arvDis": 5.0,
            "pointTag": "loc",
            "antiDis": 5.0,
            "loops": 1
        }

        # 默认目标点
        default_destination = [{"x_lat": 100.0, "y_lon": 50.0, "z_alt": -25.0}]

        # 默认航点
        default_waypoints = [
            {"x_lat": 30.0, "y_lon": 20.0, "z_alt": -25.0},
            {"x_lat": 60.0, "y_lon": 40.0, "z_alt": -25.0},
            {"x_lat": 90.0, "y_lon": 60.0, "z_alt": -25.0},
            {"x_lat": 120.0, "y_lon": 80.0, "z_alt": -25.0}
        ]

        # 默认搜索区域
        default_area_points = [
            {"x_lat": 50.0, "y_lon": 25.0, "z_alt": -30.0},
            {"x_lat": 100.0, "y_lon": 50.0, "z_alt": -30.0},
            {"x_lat": 150.0, "y_lon": 75.0, "z_alt": -30.0},
            {"x_lat": 200.0, "y_lon": 100.0, "z_alt": -30.0}
        ]

        # 编队偏移
        default_formation_offset = {
            "diff_x_lat": 0.0,
            "diff_y_lon": 0.0,
            "diff_z_alt": 0.0
        }

        defaults = {
            "SetLine": {
                **base_params,
                "wayPoints": default_waypoints,
                "formOffset": default_formation_offset,
                "groupid": 1
            },

            "LockCtrl": {
                **base_params,
                "dstLoc": default_destination,
                "alt": -30.0
            },

            "TakeOff": {
                **base_params,
                "alt": -30.0,
                "dstLoc": default_destination,
                "pointTag": "loc"
            },

            "GoToDst": {
                **base_params,
                "dstLoc": default_destination,
                "pointTag": "loc",
                "spd": 8.0,
                "arvDis": 5.0
            },

            "AutoTrace": {
                **base_params,
                "spd": 6.0,
                "arvDis": 2.0,
                "antiDis": 8.0
            },

            "TraceAttack": {
                **base_params,
                "spd": 8.0,
                "arvDis": 3.0,
                "antiDis": 10.0,
                "traceType": 0,  # 0: 跟踪, 1: 攻击
                "targetId": 101
            },

            "PatternSearch": {
                **base_params,
                "areaPoints": default_area_points,
                "pointTag": "loc",
                "spd": 6.0,
                "arvDis": 8.0,
                "antiDis": 8.0
            },

            "WaypointMission": {
                **base_params,
                "wayPoints": default_waypoints,
                "pointTag": "loc",
                "loops": 2,
                "spd": 6.0,
                "arvDis": 2.0
            },

            "FormFly": {
                **base_params,
                "wayPoints": [
                    {"x_lat": 23.661289788690084, "y_lon": 107.02720853967423, "z_alt": -30.0},
                    {"x_lat": 23.659265430554882, "y_lon": 107.03040573281862, "z_alt": -30.0}
                ],
                "pointTag": "gps",
                "formOffset": default_formation_offset,
                "spd": 10.0,
                "arvDis": 1.0,
                "groupid": 130
            },

            "JoyStick": {
                **base_params,
                "spd": 5.0,
                "antiDis": 5.0
            },

            "Land": {
                **base_params,
                "antiDis": 3.0,
                "alt": -5.0
            },

            "Hover": {
                **base_params,
                "antiDis": 2.0,
                "spd": 3.0,
                "alt": -25.0
            },

            "Emergency": {
                **base_params,
                "emergencyAlt": -2.0,
                "antiDis": 3.0
            },

            "ComprehensiveMission": {
                **base_params,
                "alt": -30.0,
                "dstLoc": default_destination,
                "wayPoints": default_waypoints,
                "pointTag": "loc",
                "spd": 8.0,
                "arvDis": 3.0,
                "loops": 1,
                "formOffset": default_formation_offset
            },

            "WayPoint": {
                **base_params,
                "wayPoints": default_waypoints,
                "pointTag": "loc",
                "loops": 1,
                "spd": 6.0,
                "arvDis": 3.0
            },

            "Search": {
                **base_params,
                "areaPoints": default_area_points,
                "pointTag": "loc",
                "spd": 6.0,
                "arvDis": 8.0,
                "antiDis": 8.0
            },

            "StatusMonitor": {
                **base_params,
                "antiDis": 3.0,
                "spd": 5.0,
                "monitorInterval": 1.0
            }
        }

        # 获取指定树的参数，如果不存在则返回基础参数
        tree_params = defaults.get(tree_name, base_params.copy())

        # 确保所有参数都有默认值
        if "dstLoc" not in tree_params and tree_name in ["GoToDst", "TakeOff", "LockCtrl"]:
            tree_params["dstLoc"] = default_destination

        if "wayPoints" not in tree_params and tree_name in ["WayPoint", "WaypointMission", "FormFly"]:
            tree_params["wayPoints"] = default_waypoints

        if "areaPoints" not in tree_params and tree_name in ["Search", "PatternSearch"]:
            tree_params["areaPoints"] = default_area_points

        if "alt" not in tree_params and tree_name in ["TakeOff", "Land", "Hover"]:
            tree_params["alt"] = -30.0

        return tree_params

    # =================== 具体测试方法 ===================

    def _test_sethome(self) -> None:
        """测试设置Home点"""
        self.send_command(
            cmd_type=6,  # CmdSetHome
            param0=int(39.9042 * 1e7),
            param1=int(116.4074 * 1e7),
            param2=int(50.0 * 1e3)
        )
        time.sleep(3.0)
        if self.current_test:
            self._complete_test(TestStatus.PASSED, "Home set command sent successfully")

    def _test_traceattack(self) -> None:
        """测试跟踪攻击"""
        self.set_simulation_state(
            locked=False,
            flight_mode=FlightMode.OFFBOARD,
            position={'x': 50.0, 'y': 25.0, 'z': -25.0, 'yaw': 0.0}
        )
        params = self._get_default_parameters("TraceAttack")
        self.send_mission_json("跟踪攻击", 8, "TraceAttack", params)

    def _test_joystickcontrol(self) -> None:
        """测试摇杆控制"""
        self.set_simulation_state(
            locked=False,
            flight_mode=FlightMode.OFFBOARD
        )
        self.vehicle_sim_state.joystick_control_active = True
        params = self._get_default_parameters("JoyStick")
        self.send_mission_json("摇杆控制", 9, "JoyStick", params)

    def _test_waypointloop(self) -> None:
        """测试多循环航点任务"""
        self.set_simulation_state(
            locked=False,
            flight_mode=FlightMode.OFFBOARD,
            position={'x': 0.0, 'y': 0.0, 'z': -25.0, 'yaw': 0.0}
        )
        params = self._get_default_parameters("WaypointMission")
        self.send_mission_json("多循环航点", 10, "WaypointMission", params)

    def _test_emergencylanding(self) -> None:
        """测试紧急降落"""
        self.set_simulation_state(
            locked=False,
            flight_mode=FlightMode.OFFBOARD,
            position={'x': 20.0, 'y': 15.0, 'z': -30.0, 'yaw': 0.0}
        )
        params = self._get_default_parameters("Emergency")
        self.send_mission_json("紧急降落", 11, "Emergency", params)

    def _test_comprehensivemission(self) -> None:
        """测试综合任务"""
        self.set_simulation_state(
            locked=False,
            position={'x': 0.0, 'y': 0.0, 'z': -5.0, 'yaw': 0.0}
        )
        params = {
            "alt": -30.0,
            "dstLoc": [{"x_lat": 120.0, "y_lon": 80.0, "z_alt": -25.0}],
            "pointTag": "loc",
            "vehiType": "多旋翼",
            "spd": 8.0,
            "arvDis": 3.0
        }
        self.send_mission_json("综合任务", 12, "ComprehensiveMission", params)

    # =================== 工具方法 ===================

    def send_mission_json(self, stage_name: str, stage_sn: int, action_name: str,
                          params: Optional[Dict[str, Any]] = None, cmd: str = "start",
                          group_id: int = 1) -> None:
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
                    "params": [],
                    "triggers": [
                        {"name": "triggleType", "type": "string", "value": "manual"},
                        {"name": "delay", "type": "float", "value": ""}
                    ]
                }]
            }]
        }

        if params:
            for key, value in params.items():
                param_entry = {
                    "name": key,
                    "type": self._get_param_type(value),
                    "value": value
                }
                mission_json["stage"][0]["actions"][0]["params"].append(param_entry)

        msg = String()
        msg.data = json.dumps(mission_json)
        self.mission_json_pub.publish(msg)

        self.get_logger().info(f"Sent {stage_name} mission (stage {stage_sn})")

    def _get_param_type(self, value: Any) -> str:
        """根据值推断参数类型"""
        if isinstance(value, str):
            return "string"
        elif isinstance(value, bool):
            return "bool"
        elif isinstance(value, int):
            return "int"
        elif isinstance(value, float):
            return "float"
        elif isinstance(value, list):
            return "line" if value and isinstance(value[0], dict) else "array"
        elif isinstance(value, dict):
            return "point"
        else:
            return "string"

    def send_command(self, cmd_type: int, **kwargs: Any) -> None:
        """发送命令"""
        cmd = CommandRequest()
        cmd.type = cmd_type
        cmd.dst = self.vehicle_id

        for key, value in kwargs.items():
            if hasattr(cmd, key):
                setattr(cmd, key, value)

        self.command_pub.publish(cmd)

    def set_simulation_state(self, locked: Optional[bool] = None,
                             flight_mode: Optional[FlightMode] = None,
                             position: Optional[Dict[str, float]] = None) -> None:
        """设置仿真状态"""
        if locked is not None:
            self.sim_locked = locked
            self.vehicle_sim_state.is_locked = locked

        if flight_mode is not None:
            self.sim_flight_mode = flight_mode.value
            self.vehicle_sim_state.flight_mode = flight_mode

        if position:
            self.sim_position.update(position)
            self.target_position.update(position)

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
        self.node_execution_count.clear()
        self.vehicle_sim_state.reset()
        self.behavior_tree_status = None
        self.node_status_history.clear()

    def run_all_tests(self) -> None:
        """运行所有测试"""
        self.get_logger().info("=" * 80)
        self.get_logger().info("STARTING BEHAVIOR NODE TEST SUITE")
        self.get_logger().info("=" * 80)

        test_cases = self.create_test_cases()
        self.test_statistics['total'] = len(test_cases)

        for i, test_case in enumerate(test_cases, 1):
            if not self.is_running:
                break

            self.get_logger().info(
                f"\n[{i}/{len(test_cases)}] Running {test_case.test_category} test: {test_case.name}")
            self.run_test_case(test_case)

            # 测试间隔
            for _ in range(20):
                rclpy.spin_once(self, timeout_sec=0.1)

        self.print_test_summary()

    def print_test_summary(self) -> None:
        """打印增强的测试总结"""
        self.get_logger().info("\n" + "=" * 80)
        self.get_logger().info("BEHAVIOR NODE TEST SUMMARY - RESULTS")
        self.get_logger().info("=" * 80)

        stats = self.test_statistics
        self.get_logger().info(f"Total Tests:        {stats['total']}")
        self.get_logger().info(f"Passed:             {stats['passed']}")
        self.get_logger().info(f"Failed:             {stats['failed']}")
        self.get_logger().info(f"Timeout:            {stats['timeout']}")
        self.get_logger().info(f"Skipped:            {stats['skipped']}")
        self.get_logger().info("-" * 40)
        self.get_logger().info(f"Basic Tests:        {stats['basic_tests']}")
        self.get_logger().info(f"Advanced Tests:     {stats['advanced_tests']}")
        self.get_logger().info(f"Error Tests:        {stats['error_tests']}")
        self.get_logger().info(f"Integration Tests:  {stats['integration_tests']}")
        self.get_logger().info("-" * 80)

        # 按类别分组显示测试结果
        categories = ["basic", "advanced", "error_handling", "integration"]
        for category in categories:
            category_tests = [test for test in self.test_results.values()
                              if test.test_category == category]
            if category_tests:
                self.get_logger().info(f"\n{category.upper()} TESTS:")
                for test in category_tests:
                    symbol = "✓" if test.status == TestStatus.PASSED else "✗"
                    duration = test.end_time - test.start_time if test.end_time > 0 else 0
                    self.get_logger().info(f"  {symbol} {test.name:25} [{test.status.value:7}] {duration:6.1f}s")

        success_rate = (stats['passed'] / stats['total'] * 100) if stats['total'] > 0 else 0
        self.get_logger().info("-" * 80)
        self.get_logger().info(f"Overall Success Rate: {success_rate:.1f}%")

        # 节点覆盖统计
        self.get_logger().info(f"\nNode Execution Coverage:")
        for node, count in self.node_execution_count.items():
            self.get_logger().info(f"  {node}: {count} executions")

        # 服务调用统计
        self.get_logger().info(f"\nService Call Statistics:")
        for service, count in self.service_call_count.items():
            self.get_logger().info(f"  {service}: {count} calls")

        if success_rate >= 90:
            self.get_logger().info("🎉 EXCELLENT TEST RESULTS!")
        elif success_rate >= 75:
            self.get_logger().info("👍 GOOD COVERAGE TEST RESULTS!")
        else:
            self.get_logger().info("⚠️  COVERAGE TESTS NEED ATTENTION")

        self.get_logger().info("=" * 80)

    def shutdown(self) -> None:
        """关闭测试器"""
        self.is_running = False
        self.get_logger().info("Behavior Node Tester shutting down...")


def main() -> None:
    """主函数"""
    parser = argparse.ArgumentParser(description='Behavior Node Tester - Full Coverage')
    parser.add_argument('--test', choices=[
        'all', 'basic', 'advanced', 'error_handling', 'integration',
        'set_home', 'parameter_config', 'unlock_vehicle', 'takeoff', 'hover',
        'goto_destination', 'auto_trace', 'trace_attack', 'pattern_search',
        'waypoint_mission', 'formation_flight', 'joystick_control', 'land',
        'comprehensive_mission', 'emergency_landing'
    ], default='all', help='Specify which test(s) to run')
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
            tester.run_all_tests()
        elif args.test in ['basic', 'advanced', 'error_handling', 'integration']:
            # 运行特定类别的测试
            test_cases = tester.create_test_cases()
            category_tests = [tc for tc in test_cases if tc.test_category == args.test]

            tester.get_logger().info(f"Running {args.test} tests ({len(category_tests)} tests)")
            for test_case in category_tests:
                if not tester.is_running:
                    break
                tester.run_test_case(test_case)
                for _ in range(20):
                    executor.spin_once(timeout_sec=0.1)

            tester.print_test_summary()
        else:
            # 运行单个测试
            test_cases = tester.create_test_cases()
            target_test = None
            for test_case in test_cases:
                if test_case.name.lower().replace(' ', '_') == args.test.replace('_', ''):
                    target_test = test_case
                    break

            if target_test:
                tester.get_logger().info(f"Running single test: {target_test.name}")
                tester.run_test_case(target_test)
                tester.print_test_summary()
            else:
                tester.get_logger().error(f"Unknown test: {args.test}")

        # 继续处理ROS消息
        for _ in range(50):
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
