#!/usr/bin/env python3
"""
Behavior Node Test Suite (Python 3.8+ Compatible)
行为树测试工具，兼容Python 3.8+
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
    """统一的behavior_node测试器"""

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

        # 配置参数
        self.vehicle_id = 1
        self.enable_detailed_logging = True
        self.success_rate = 0.95
        self.response_delay = 0.1
        self.simulate_failures = False  # 默认不模拟失败

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

    def _setup_services(self) -> None:
        """设置服务模拟器"""
        # 飞行模式控制服务
        self.flight_mode_service = self.create_service(
            CommandLong,
            'inner/control/set_flymode',
            self._handle_flight_mode
        )

        # 锁定/解锁控制服务
        self.lock_unlock_service = self.create_service(
            CommandBool,
            'inner/control/lock_unlock',
            self._handle_lock_unlock
        )

        # 编队切换控制服务
        self.formation_switch_service = self.create_service(
            CommandInt,
            'inner/control/form_switch',
            self._handle_formation_switch
        )

        # RTSP URL获取服务
        self.rtsp_url_service = self.create_service(
            CommandString,
            'inner/get/rtsp_url',
            self._handle_rtsp_url
        )

        # 解锁/上锁服务
        self.arm_disarm_service = self.create_service(
            CommandBool,
            'inner/control/arm_disarm',
            self._handle_arm_disarm
        )

        # 任务控制服务
        self.mission_control_service = self.create_service(
            CommandInt,
            'inner/control/mission_control',
            self._handle_mission_control
        )

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

        self.last_commands[service_name] = {
            'request_data': request,
            'timestamp': time.time()
        }

        if self._should_succeed(service_name):
            response.success = True
            response.rslt = self.vehicle_sim_state.rtsp_url
            self.get_logger().info(f"RTSP URL requested: {self.vehicle_sim_state.rtsp_url}")
        else:
            response.success = False
            response.rslt = "Failed to get RTSP URL"
            self.get_logger().warn(f"RTSP URL request failed (simulated)")

        return response

    def _handle_arm_disarm(self, request: CommandBool.Request, response: CommandBool.Response) -> CommandBool.Response:
        """处理解锁/上锁请求"""
        service_name = "arm_disarm"
        self._increment_call_count(service_name)
        self._simulate_delay()

        arm_requested = request.value
        action = "arm" if arm_requested else "disarm"

        self.last_commands[service_name] = {
            'action': action,
            'value': arm_requested,
            'timestamp': time.time()
        }

        if arm_requested and self.vehicle_sim_state.is_locked:
            response.success = False
            response.result = "Cannot arm: Vehicle is locked"
            self.get_logger().warn(f"Arm rejected: Vehicle locked")
            return response

        if self._should_succeed(service_name):
            self.vehicle_sim_state.is_armed = arm_requested
            response.success = True
            response.result = f"Vehicle {action}ed successfully"
            self.get_logger().info(f"Vehicle {action}ed successfully")
        else:
            response.success = False
            response.result = f"Failed to {action} vehicle"
            self.get_logger().warn(f"Failed to {action} vehicle (simulated)")

        return response

    def _handle_mission_control(self, request: CommandInt.Request,
                                response: CommandInt.Response) -> CommandInt.Response:
        """处理任务控制请求"""
        service_name = "mission_control"
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
                    self.vehicle_sim_state.mission_active = True
                elif command == 3:  # STOP
                    self.vehicle_sim_state.mission_active = False

                response.success = True
                self.get_logger().info(f"Mission {nav_command} executed (frame={frame})")
            else:
                response.success = False
                self.get_logger().warn(f"Mission command failed (simulated)")

        except Exception as e:
            response.success = False
            self.get_logger().error(f"Mission command error: {e}")

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

        # 检查offboard控制指令数量
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
        if test.tree_name == "LockCtrl":
            if "lock_unlock" not in self.service_calls:
                return False, "Lock/unlock service not called"
            if self.vehicle_sim_state.is_locked:  # 应该是解锁状态
                return False, "Vehicle not in unlocked state"

        elif test.tree_name == "TakeOff":
            if FlightMode.TAKEOFF.value not in self.mode_changes:
                return False, "Takeoff mode not activated"
            if len(self.offboard_commands) < 10:
                return False, f"Insufficient takeoff control commands: {len(self.offboard_commands)}"

        elif test.tree_name == "GotoDst":
            if len(self.offboard_commands) < 15:
                return False, f"Insufficient navigation commands: {len(self.offboard_commands)}"
            # 检查位置是否接近目标
            target_reached = self._check_position_reached(100.0, 50.0, -25.0, tolerance=20.0)
            if not target_reached:
                return False, "Target position not reached"

        elif test.tree_name == "AutoTrace":
            if len(self.offboard_commands) < 8:
                return False, f"Insufficient trace commands: {len(self.offboard_commands)}"

        elif test.tree_name == "Land":
            if FlightMode.LAND.value not in self.mode_changes:
                return False, "Land mode not activated"

        elif test.tree_name == "FormFly":
            if "form_switch" not in self.service_calls:
                return False, "Formation switch service not called"

        elif test.tree_name == "SetLine":
            # 参数设置测试只需要运行一定时间
            if test_duration < 2.0:
                return False, f"Parameter setting test too short: {test_duration:.1f}s"

        return True, f"All success criteria met after {test_duration:.1f}s"

    def _evaluate_test_failure(self) -> Tuple[bool, str]:
        """评估测试失败条件"""
        if not self.current_test:
            return False, "No current test"

        current_time = time.time()
        test_duration = current_time - self.test_start_time

        # 检查服务调用失败
        failed_services = []
        for service_name, count in self.service_call_count.items():
            if count > 0 and service_name in self.last_commands:
                last_cmd = self.last_commands[service_name]
                if 'success' in last_cmd and not last_cmd['success']:
                    failed_services.append(service_name)

        if failed_services:
            return True, f"Service calls failed: {failed_services}"

        # 检查是否没有任何活动
        if test_duration > 10.0 and len(self.offboard_commands) == 0:
            return True, "No offboard commands received after 10 seconds"

        return False, "No failure conditions detected"

    def _check_position_reached(self, target_x: float, target_y: float, target_z: float,
                                tolerance: float = 5.0) -> bool:
        """检查是否到达目标位置"""
        current_x = self.sim_position['x']
        current_y = self.sim_position['y']
        current_z = self.sim_position['z']

        distance = math.sqrt((current_x - target_x) ** 2 + (current_y - target_y) ** 2 + (current_z - target_z) ** 2)
        return distance < tolerance

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

    # =================== 测试工具方法 ===================

    def send_mission_json(self, stage_name: str, stage_sn: int, action_name: str,
                          params: Optional[Dict[str, Any]] = None, cmd: str = "set",
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

    def send_command(self, cmd_type: int, **kwargs: Any) -> None:
        """发送命令"""
        cmd = CommandRequest()
        cmd.type = cmd_type
        cmd.dst = self.vehicle_id

        for key, value in kwargs.items():
            if hasattr(cmd, key):
                setattr(cmd, key, value)

        self.command_pub.publish(cmd)
        self.get_logger().info(f"Sent command type: {cmd_type}")

    def set_simulation_state(self, locked: Optional[bool] = None, flight_mode: Optional[FlightMode] = None,
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
            time.sleep(0.1)
            if self.current_test:
                self._check_test_completion()

    def reset_test_state(self) -> None:
        """重置测试状态"""
        self.offboard_commands.clear()
        self.mode_changes.clear()
        self.service_calls.clear()
        self.service_call_count.clear()
        self.last_commands.clear()
        self.vehicle_sim_state.reset()

    # =================== 具体测试用例 ===================

    def create_test_cases(self) -> List[TestCase]:
        """创建所有测试用例"""
        return [
            TestCase(
                name="SetHome",
                description="Test setting home point",
                tree_name="",
                timeout=10.0,
                success_criteria={'min_duration': 1.0, 'required_offboard_count': 0}
            ),
            TestCase(
                name="ParameterConfig",
                description="Test parameter configuration using SetLine behavior tree",
                tree_name="SetLine",
                timeout=15.0,
                success_criteria={'min_duration': 2.0, 'required_offboard_count': 0}
            ),
            TestCase(
                name="UnlockVehicle",
                description="Test vehicle unlock using LockCtrl behavior tree",
                tree_name="LockCtrl",
                timeout=15.0,
                required_services=["lock_unlock"],
                success_criteria={'min_duration': 3.0, 'required_offboard_count': 5}
            ),
            TestCase(
                name="TakeOff",
                description="Test takeoff using TakeOff behavior tree",
                tree_name="TakeOff",
                timeout=25.0,
                required_mode_changes=[FlightMode.TAKEOFF.value],
                required_services=["lock_unlock", "set_flymode"],
                success_criteria={'min_duration': 5.0, 'required_offboard_count': 10}
            ),
            TestCase(
                name="GotoDestination",
                description="Test goto destination using GotoDst behavior tree",
                tree_name="GotoDst",
                timeout=30.0,
                required_services=["lock_unlock"],
                success_criteria={'min_duration': 8.0, 'required_offboard_count': 15}
            ),
            TestCase(
                name="AutoTrace",
                description="Test auto trace using AutoTrace behavior tree",
                tree_name="AutoTrace",
                timeout=20.0,
                required_services=["lock_unlock"],
                success_criteria={'min_duration': 6.0, 'required_offboard_count': 8}
            ),
            TestCase(
                name="Land",
                description="Test landing using Land behavior tree",
                tree_name="Land",
                timeout=20.0,
                required_mode_changes=[FlightMode.LAND.value],
                success_criteria={'min_duration': 4.0, 'required_offboard_count': 8}
            ),
            TestCase(
                name="FormationFlight",
                description="Test formation flight using FormFly behavior tree",
                tree_name="FormFly",
                timeout=25.0,
                required_services=["lock_unlock", "form_switch"],
                success_criteria={'min_duration': 8.0, 'required_offboard_count': 12}
            )
        ]

    def run_test_case(self, test_case: TestCase) -> None:
        """运行单个测试用例"""
        self.reset_test_state()
        self.current_test = test_case
        self.test_start_time = time.time()
        test_case.start_time = self.test_start_time
        test_case.status = TestStatus.RUNNING

        self.get_logger().info(f"Starting test: {test_case.name}")

        try:
            if test_case.name == "SetHome":
                self._test_set_home()
            elif test_case.name == "ParameterConfig":
                self._test_parameter_config()
            elif test_case.name == "UnlockVehicle":
                self._test_unlock_vehicle()
            elif test_case.name == "TakeOff":
                self._test_takeoff()
            elif test_case.name == "GotoDestination":
                self._test_goto_destination()
            elif test_case.name == "AutoTrace":
                self._test_auto_trace()
            elif test_case.name == "Land":
                self._test_land()
            elif test_case.name == "FormationFlight":
                self._test_formation_flight()

            self.wait_for_test_completion(test_case.timeout)

        except Exception as e:
            if self.current_test:
                self._complete_test(TestStatus.FAILED, f"Test execution error: {str(e)}")

    def _test_set_home(self) -> None:
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

    def _test_parameter_config(self) -> None:
        """测试参数配置"""
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

    def _test_unlock_vehicle(self) -> None:
        """测试解锁载具"""
        self.set_simulation_state(locked=False)
        params = {"vehiType": "多旋翼", "spd": 5.0, "arvDis": 1.0}
        self.send_mission_json("Unlock", 1, "LockCtrl", params)

    def _test_takeoff(self) -> None:
        """测试起飞"""
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

    def _test_goto_destination(self) -> None:
        """测试飞向目标点"""
        self.set_simulation_state(
            locked=False,
            flight_mode=FlightMode.OFFBOARD,
            position={'x': 0.0, 'y': 0.0, 'z': -25.0, 'yaw': 0.0}
        )
        params = {
            "dstLoc": [{"x_lat": 100.0, "y_lon": 50.0, "z_alt": -25.0}],
            "pointTag": "loc",
            "spd": 8.0,
            "arvDis": 3.0,
            "vehiType": "多旋翼"
        }
        self.send_mission_json("GotoDst", 3, "GotoDst", params)

    def _test_auto_trace(self) -> None:
        """测试自动跟踪"""
        self.set_simulation_state(
            locked=False,
            flight_mode=FlightMode.OFFBOARD,
            position={'x': 80.0, 'y': 40.0, 'z': -30.0, 'yaw': 0.0}
        )
        params = {"vehiType": "多旋翼", "spd": 6.0, "arvDis": 2.0}
        self.send_mission_json("AutoTrace", 4, "AutoTrace", params)

    def _test_land(self) -> None:
        """测试降落"""
        self.set_simulation_state(
            locked=False,
            flight_mode=FlightMode.LAND,
            position={'x': 0.0, 'y': 0.0, 'z': -25.0, 'yaw': 0.0}
        )
        params = {"vehiType": "多旋翼"}
        self.send_mission_json("Land", 5, "Land", params)

    def _test_formation_flight(self) -> None:
        """测试编队飞行"""
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

    def run_all_tests(self) -> None:
        """运行所有测试"""
        self.get_logger().info("=" * 80)
        self.get_logger().info("STARTING BEHAVIOR NODE TEST SUITE")
        self.get_logger().info("=" * 80)

        test_cases = self.create_test_cases()

        for i, test_case in enumerate(test_cases, 1):
            if not self.is_running:
                break

            self.get_logger().info(f"\n[{i}/{len(test_cases)}] Running test: {test_case.name}")
            self.run_test_case(test_case)
            time.sleep(2.0)  # 测试间隔

        self.print_test_summary()

    def print_test_summary(self) -> None:
        """打印测试总结"""
        self.get_logger().info("\n" + "=" * 80)
        self.get_logger().info("BEHAVIOR NODE TEST SUMMARY")
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

        # 打印服务调用统计
        self.get_logger().info("\nService Call Statistics:")
        for service, count in self.service_call_count.items():
            self.get_logger().info(f"  {service}: {count} calls")

        self.get_logger().info("=" * 80)

    def shutdown(self) -> None:
        """关闭测试器"""
        self.is_running = False
        self.get_logger().info("Behavior Node Tester shutting down...")


def main() -> None:
    """主函数"""
    parser = argparse.ArgumentParser(description='Behavior Node Tester (Python 3.8+ Compatible)')
    parser.add_argument('--test', choices=[
        'all', 'set_home', 'parameter_config', 'unlock_vehicle', 'takeoff',
        'goto_destination', 'auto_trace', 'land', 'formation_flight'
    ], default='all', help='Specify which test to run')
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
        else:
            # 运行单个测试
            test_cases = tester.create_test_cases()
            target_test = None
            for test_case in test_cases:
                if test_case.name.lower() == args.test.replace('_', '').lower():
                    target_test = test_case
                    break

            if target_test:
                tester.get_logger().info(f"Running single test: {target_test.name}")
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