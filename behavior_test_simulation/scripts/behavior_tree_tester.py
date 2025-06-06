#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from custom_msgs.msg import CommandRequest, StatusTask, SimpleVehicle
from geometry_msgs.msg import Point
import json
import time
import sys
import threading
from typing import Dict, List, Any

class BehaviorTreeTester(Node):
    def __init__(self):
        super().__init__('behavior_tree_tester')

        # 发布器
        self.mission_json_pub = self.create_publisher(String, 'outer/set/stage_info_json', 10)
        self.command_pub = self.create_publisher(CommandRequest, 'outer/command/request', 10)

        # 订阅器
        self.status_sub = self.create_subscription(
            StatusTask, 'outer/information/status_task', self.status_callback, 10)
        self.vehicle_sub = self.create_subscription(
            SimpleVehicle, 'inner/information/simple_vehicle', self.vehicle_callback, 10)

        # 测试状态
        self.current_test = None
        self.test_results = {}
        self.vehicle_state = None
        self.task_status = None
        self.test_start_time = None

        # 测试超时时间 (秒)
        self.test_timeout = 30.0

        self.get_logger().info("Behavior Tree Tester initialized")

    def status_callback(self, msg):
        self.task_status = msg
        self.get_logger().info(f"Task status: stage={msg.stage}, id={msg.id}, status={msg.status}")

        if self.current_test:
            self.evaluate_test_progress()

    def vehicle_callback(self, msg):
        self.vehicle_state = msg

    def evaluate_test_progress(self):
        """评估测试进度"""
        if not self.current_test or not self.task_status:
            return

        test_name = self.current_test['name']
        expected_stage = self.current_test.get('expected_stage', -1)

        # 检查是否超时
        if time.time() - self.test_start_time > self.test_timeout:
            self.complete_test(test_name, False, "Test timeout")
            return

        # 检查任务状态
        if self.task_status.stage == expected_stage:
            if self.task_status.status == 3:  # StsComplete
                self.complete_test(test_name, True, "Mission completed successfully")
            elif self.task_status.status == 2:  # StsFailed
                self.complete_test(test_name, False, "Mission failed")

    def complete_test(self, test_name: str, success: bool, message: str):
        """完成测试"""
        duration = time.time() - self.test_start_time if self.test_start_time else 0

        self.test_results[test_name] = {
            'success': success,
            'message': message,
            'duration': duration
        }

        status = "PASSED" if success else "FAILED"
        self.get_logger().info(f"Test '{test_name}' {status}: {message} (Duration: {duration:.1f}s)")

        self.current_test = None
        self.test_start_time = None

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
                    "id": 1,
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

    def send_command(self, cmd_type: int, dst: int = 1, **kwargs):
        """发送命令"""
        cmd = CommandRequest()
        cmd.type = cmd_type
        cmd.dst = dst

        for key, value in kwargs.items():
            if hasattr(cmd, key):
                setattr(cmd, key, value)

        self.command_pub.publish(cmd)

    def wait_for_test_completion(self, max_wait_time: float = 30.0):
        """等待测试完成"""
        start_time = time.time()
        rate = self.create_rate(10)  # 10 Hz

        while rclpy.ok() and self.current_test:
            if time.time() - start_time > max_wait_time:
                self.complete_test(self.current_test['name'], False, "Wait timeout")
                break
            rate.sleep()

    def test_set_home(self):
        """测试设置Home点"""
        test_name = "SetHome"
        self.get_logger().info(f"Starting test: {test_name}")

        self.current_test = {'name': test_name}
        self.test_start_time = time.time()

        # 发送设置Home命令
        self.send_command(
            cmd_type=6,  # CmdSetHome
            param0=int(39.9042 * 1e7),  # 纬度
            param1=int(116.4074 * 1e7),  # 经度
            param2=int(50.0 * 1e3)  # 高度
        )

        # 等待一段时间检查结果
        time.sleep(2.0)
        self.complete_test(test_name, True, "Home set command sent")

    def test_takeoff(self):
        """测试起飞"""
        test_name = "TakeOff"
        self.get_logger().info(f"Starting test: {test_name}")

        self.current_test = {'name': test_name, 'expected_stage': 1}
        self.test_start_time = time.time()

        params = {
            "alt": 30.0,
            "pointTag": "loc"
        }

        self.send_mission_json("TakeOff", 1, "TakeOff", params)
        self.wait_for_test_completion(15.0)

    def test_goto_destination(self):
        """测试飞向目标点"""
        test_name = "GotoDestination"
        self.get_logger().info(f"Starting test: {test_name}")

        self.current_test = {'name': test_name, 'expected_stage': 2}
        self.test_start_time = time.time()

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
        self.wait_for_test_completion(20.0)

    def test_auto_trace(self):
        """测试自动跟踪"""
        test_name = "AutoTrace"
        self.get_logger().info(f"Starting test: {test_name}")

        self.current_test = {'name': test_name, 'expected_stage': 3}
        self.test_start_time = time.time()

        self.send_mission_json("AutoTrace", 3, "AutoTrace")

        # 自动跟踪测试运行5秒后停止
        time.sleep(5.0)
        self.send_mission_json("Stop", 3, "Stop", cmd="del")

        self.complete_test(test_name, True, "AutoTrace test completed")

    def test_land(self):
        """测试降落"""
        test_name = "Land"
        self.get_logger().info(f"Starting test: {test_name}")

        self.current_test = {'name': test_name, 'expected_stage': 4}
        self.test_start_time = time.time()

        self.send_mission_json("Land", 4, "Land")
        self.wait_for_test_completion(10.0)

    def run_all_tests(self):
        """运行所有测试"""
        self.get_logger().info("Starting comprehensive behavior tree test suite...")

        tests = [
            self.test_set_home,
            self.test_takeoff,
            self.test_goto_destination,
            self.test_auto_trace,
            self.test_land
        ]

        for test_func in tests:
            if not rclpy.ok():
                break

            try:
                test_func()
                time.sleep(2.0)  # 测试间隔
            except Exception as e:
                test_name = test_func.__name__
                self.get_logger().error(f"Test {test_name} failed with exception: {str(e)}")
                self.test_results[test_name] = {
                    'success': False,
                    'message': f"Exception: {str(e)}",
                    'duration': 0
                }

        self.print_test_summary()

    def print_test_summary(self):
        """打印测试总结"""
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("BEHAVIOR TREE TEST SUMMARY")
        self.get_logger().info("="*60)

        total_tests = len(self.test_results)
        passed_tests = sum(1 for result in self.test_results.values() if result['success'])

        for test_name, result in self.test_results.items():
            status = "PASSED" if result['success'] else "FAILED"
            duration = result['duration']
            message = result['message']
            self.get_logger().info(f"{test_name:20} [{status:6}] {duration:6.1f}s - {message}")

        self.get_logger().info("-" * 60)
        self.get_logger().info(f"Total Tests: {total_tests}")
        self.get_logger().info(f"Passed:     {passed_tests}")
        self.get_logger().info(f"Failed:     {total_tests - passed_tests}")
        self.get_logger().info(f"Success Rate: {passed_tests/total_tests*100:.1f}%" if total_tests > 0 else "No tests run")
        self.get_logger().info("="*60)

def main():
    rclpy.init()

    tester = BehaviorTreeTester()

    # 等待系统启动
    tester.get_logger().info("Waiting for system to initialize...")
    time.sleep(3.0)

    try:
        if len(sys.argv) > 1 and sys.argv[1] == "--interactive":
            # 交互式模式
            tester.get_logger().info("Interactive test mode")
            rclpy.spin(tester)
        else:
            # 自动测试模式
            test_thread = threading.Thread(target=tester.run_all_tests)
            test_thread.start()

            rclpy.spin(tester)
            test_thread.join()

    except KeyboardInterrupt:
        tester.get_logger().info("Test interrupted by user")
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()