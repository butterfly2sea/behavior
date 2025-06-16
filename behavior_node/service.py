#!/usr/bin/env python3
"""
Behavior Node Service Simulator
为behavior_node提供完整的service接口模拟，用于单独测试
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import time
import threading
import json
from enum import Enum
from typing import Dict, List, Any, Optional
from dataclasses import dataclass

# ROS2 服务类型
from custom_msgs.srv import CommandBool, CommandLong, CommandInt, CommandString


class FlightMode(Enum):
    """飞行模式枚举"""
    UNKNOWN = 0
    MANUAL = 1  # 手动
    ALT = 2  # 定高
    POS = 3  # 定点
    TAKEOFF = 4  # 起飞
    LAND = 5  # 降落
    HOLD = 6  # 等待
    MISSION = 7  # 任务
    RTL = 8  # 返航
    OFFBOARD = 9  # 外部
    STABLE = 10  # 增稳


class LockState(Enum):
    """锁定状态枚举"""
    LOCKED = 0  # 锁定
    UNLOCKED = 1  # 解锁


class NavCommand(Enum):
    """导航命令枚举"""
    START = 0  # 开始导航
    PAUSE = 1  # 暂停导航
    RESUME = 2  # 恢复导航
    STOP = 3  # 停止导航


@dataclass
class VehicleSimState:
    """飞机仿真状态"""
    is_armed: bool = False
    is_locked: bool = True
    flight_mode: FlightMode = FlightMode.MANUAL
    formation_active: bool = False
    mission_active: bool = False
    rtsp_url: str = "rtsp://192.168.1.100:8554/camera"

    def reset(self):
        """重置状态"""
        self.is_armed = False
        self.is_locked = True
        self.flight_mode = FlightMode.MANUAL
        self.formation_active = False
        self.mission_active = False


class BehaviorNodeServiceSimulator(Node):
    """Behavior Node服务模拟器"""

    def __init__(self):
        super().__init__('behavior_service_simulator')

        # 仿真状态
        self.vehicle_state = VehicleSimState()
        self.service_call_count: Dict[str, int] = {}
        self.last_commands: Dict[str, Any] = {}

        # 配置参数
        self.success_rate = 0.95  # 成功率模拟
        self.response_delay = 0.1  # 响应延迟
        self.simulate_failures = True  # 是否模拟失败

        # 创建所有服务
        self._create_services()

        # 状态发布定时器
        self.status_timer = self.create_timer(1.0, self._publish_status)

        self.get_logger().info("Behavior Node Service Simulator started")
        self._print_service_list()

    def _create_services(self):
        """创建所有服务"""
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

    def _print_service_list(self):
        """打印服务列表"""
        services = [
            "inner/control/set_flymode (CommandLong)",
            "inner/control/lock_unlock (CommandBool)",
            "inner/control/form_switch (CommandInt)",
            "inner/get/rtsp_url (CommandString)",
            "inner/control/arm_disarm (CommandBool)",
            "inner/control/mission_control (CommandInt)"
        ]

        self.get_logger().info("Available services:")
        for service in services:
            self.get_logger().info(f"  - {service}")

    def _simulate_delay(self):
        """模拟服务响应延迟"""
        if self.response_delay > 0:
            time.sleep(self.response_delay)

    def _should_succeed(self, service_name: str) -> bool:
        """判断服务是否应该成功"""
        if not self.simulate_failures:
            return True

        import random
        return random.random() < self.success_rate

    def _increment_call_count(self, service_name: str):
        """增加服务调用计数"""
        self.service_call_count[service_name] = self.service_call_count.get(service_name, 0) + 1

    def _handle_flight_mode(self, request: CommandLong.Request, response: CommandLong.Response):
        """处理飞行模式控制请求"""
        service_name = "set_flymode"
        self._increment_call_count(service_name)
        self._simulate_delay()

        try:
            flight_mode = FlightMode(request.command)

            # 记录参数
            takeoff_altitude = request.param7

            self.last_commands[service_name] = {
                'mode': flight_mode.name,
                'command': request.command,
                'param7': takeoff_altitude,
                'timestamp': time.time()
            }


            if flight_mode == FlightMode.TAKEOFF and self.vehicle_state.is_locked:
                response.success = False
                response.result = "Cannot takeoff: Vehicle is locked"
                self.get_logger().warn(f"Takeoff rejected: Vehicle locked")
                return response

            # 模拟成功/失败
            if self._should_succeed(service_name):
                self.vehicle_state.flight_mode = flight_mode
                response.success = True
                response.result = 0

                if flight_mode == FlightMode.TAKEOFF:
                    response.result += 1

                self.get_logger().info(
                    f"Flight mode changed to {flight_mode.name} (cmd={request.command})")

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

    def _handle_lock_unlock(self, request: CommandBool.Request, response: CommandBool.Response):
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

        # 模拟成功/失败
        if self._should_succeed(service_name):
            self.vehicle_state.is_locked = not unlock_requested
            response.success = True
            response.result = True

            self.get_logger().info(f"Vehicle {action}ed successfully")
        else:
            response.success = False
            response.result = False
            self.get_logger().warn(f"Failed to {action} vehicle (simulated)")

        return response

    def _handle_formation_switch(self, request: CommandInt.Request, response: CommandInt.Response):
        """处理编队切换控制请求"""
        service_name = "form_switch"
        self._increment_call_count(service_name)
        self._simulate_delay()

        frame = request.frame
        command = request.command

        try:
            nav_command = NavCommand(command)

            self.last_commands[service_name] = {
                'frame': frame,
                'command': nav_command.name,
                'command_value': command,
                'timestamp': time.time()
            }

            # 模拟成功/失败
            if self._should_succeed(service_name):
                if nav_command == NavCommand.START:
                    self.vehicle_state.formation_active = True
                elif nav_command == NavCommand.STOP:
                    self.vehicle_state.formation_active = False

                response.success = True
                # response.result = f"Formation {nav_command.name} command executed"

                frame_desc = "自主控制" if frame == 1 else "仅计算"
                self.get_logger().info(
                    f"Formation {nav_command.name} executed (frame={frame}: {frame_desc})")
            else:
                response.success = False
                # response.result = f"Formation command failed"
                self.get_logger().warn(f"Formation command failed (simulated)")

        except ValueError:
            response.success = False
            # response.result = f"Invalid formation command: {command}"
            self.get_logger().error(f"Invalid formation command: {command}")

        return response

    def _handle_rtsp_url(self, request: CommandString.Request, response: CommandString.Response):
        """处理RTSP URL获取请求"""
        service_name = "rtsp_url"
        self._increment_call_count(service_name)
        self._simulate_delay()

        self.last_commands[service_name] = {
            'request_data': request,
            'timestamp': time.time()
        }

        # 模拟成功/失败
        if self._should_succeed(service_name):
            response.success = True
            response.rslt = self.vehicle_state.rtsp_url

            self.get_logger().info(f"RTSP URL requested: {self.vehicle_state.rtsp_url}")
        else:
            response.success = False
            response.rslt = "Failed to get RTSP URL"
            self.get_logger().warn(f"RTSP URL request failed (simulated)")

        return response

    def _handle_arm_disarm(self, request: CommandBool.Request, response: CommandBool.Response):
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

        if arm_requested and self.vehicle_state.is_locked:
            response.success = False
            response.result = "Cannot arm: Vehicle is locked"
            self.get_logger().warn(f"Arm rejected: Vehicle locked")
            return response

        # 模拟成功/失败
        if self._should_succeed(service_name):
            self.vehicle_state.is_armed = arm_requested
            response.success = True
            response.result = f"Vehicle {action}ed successfully"

            self.get_logger().info(f"Vehicle {action}ed successfully")
        else:
            response.success = False
            response.result = f"Failed to {action} vehicle"
            self.get_logger().warn(f"Failed to {action} vehicle (simulated)")

        return response

    def _handle_mission_control(self, request: CommandInt.Request, response: CommandInt.Response):
        """处理任务控制请求"""
        service_name = "mission_control"
        self._increment_call_count(service_name)
        self._simulate_delay()

        frame = request.frame
        command = request.command

        try:
            nav_command = NavCommand(command)

            self.last_commands[service_name] = {
                'frame': frame,
                'command': nav_command.name,
                'command_value': command,
                'timestamp': time.time()
            }

            # 模拟成功/失败
            if self._should_succeed(service_name):
                if nav_command == NavCommand.START:
                    self.vehicle_state.mission_active = True
                elif nav_command == NavCommand.STOP:
                    self.vehicle_state.mission_active = False

                response.success = True
                # response.result = f"Mission {nav_command.name} command executed"

                self.get_logger().info(f"Mission {nav_command.name} executed (frame={frame})")
            else:
                response.success = False
                # response.result = f"Mission command failed"
                self.get_logger().warn(f"Mission command failed (simulated)")

        except ValueError:
            response.success = False
            # response.result = f"Invalid mission command: {command}"
            self.get_logger().error(f"Invalid mission command: {command}")

        return response

    def _publish_status(self):
        """发布当前状态"""
        status_info = {
            'armed': self.vehicle_state.is_armed,
            'locked': self.vehicle_state.is_locked,
            'flight_mode': self.vehicle_state.flight_mode.name,
            'formation_active': self.vehicle_state.formation_active,
            'mission_active': self.vehicle_state.mission_active
        }

        if self.service_call_count:
            self.get_logger().info(
                f"Status: {status_info} | Calls: {sum(self.service_call_count.values())}")

    def reset_state(self):
        """重置仿真状态"""
        self.vehicle_state.reset()
        self.service_call_count.clear()
        self.last_commands.clear()
        self.get_logger().info("Simulator state reset")

    def set_success_rate(self, rate: float):
        """设置成功率"""
        self.success_rate = max(0.0, min(1.0, rate))
        self.get_logger().info(f"Success rate set to {self.success_rate:.2%}")

    def enable_failures(self, enable: bool):
        """启用/禁用失败模拟"""
        self.simulate_failures = enable
        status = "enabled" if enable else "disabled"
        self.get_logger().info(f"Failure simulation {status}")

    def set_response_delay(self, delay: float):
        """设置响应延迟"""
        self.response_delay = max(0.0, delay)
        self.get_logger().info(f"Response delay set to {self.response_delay:.2f}s")

    def get_statistics(self) -> Dict[str, Any]:
        """获取统计信息"""
        return {
            'service_calls': dict(self.service_call_count),
            'last_commands': dict(self.last_commands),
            'current_state': {
                'armed': self.vehicle_state.is_armed,
                'locked': self.vehicle_state.is_locked,
                'flight_mode': self.vehicle_state.flight_mode.name,
                'formation_active': self.vehicle_state.formation_active,
                'mission_active': self.vehicle_state.mission_active
            },
            'config': {
                'success_rate': self.success_rate,
                'simulate_failures': self.simulate_failures,
                'response_delay': self.response_delay
            }
        }

    def print_statistics(self):
        """打印统计信息"""
        stats = self.get_statistics()

        self.get_logger().info("\n" + "=" * 50)
        self.get_logger().info("SERVICE SIMULATOR STATISTICS")
        self.get_logger().info("=" * 50)

        self.get_logger().info("Service Call Counts:")
        for service, count in stats['service_calls'].items():
            self.get_logger().info(f"  {service}: {count}")

        self.get_logger().info(f"\nCurrent State:")
        for key, value in stats['current_state'].items():
            self.get_logger().info(f"  {key}: {value}")

        self.get_logger().info(f"\nConfiguration:")
        for key, value in stats['config'].items():
            self.get_logger().info(f"  {key}: {value}")

        self.get_logger().info("=" * 50)


def main():
    """主函数"""
    import argparse

    parser = argparse.ArgumentParser(description='Behavior Node Service Simulator')
    parser.add_argument('--success-rate', type=float, default=0.95,
                        help='Service success rate (0.0-1.0)')
    parser.add_argument('--no-failures', action='store_true',
                        help='Disable failure simulation')
    parser.add_argument('--delay', type=float, default=0.1,
                        help='Response delay in seconds')
    parser.add_argument('--stats-interval', type=float, default=60.0,
                        help='Statistics print interval')

    args = parser.parse_args()

    rclpy.init()

    simulator = BehaviorNodeServiceSimulator()

    # 配置参数
    simulator.set_success_rate(args.success_rate)
    simulator.enable_failures(not args.no_failures)
    simulator.set_response_delay(args.delay)

    # 统计信息定时器
    if args.stats_interval > 0:
        stats_timer = simulator.create_timer(args.stats_interval, simulator.print_statistics)

    try:
        simulator.get_logger().info("Service simulator ready. Press Ctrl+C to exit.")
        rclpy.spin(simulator)
    except KeyboardInterrupt:
        simulator.get_logger().info("Shutting down service simulator...")
    finally:
        simulator.print_statistics()
        simulator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()