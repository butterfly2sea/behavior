# Behavior Node 测试仿真系统

这是一个专门为 `behavior_node` 行为树节点程序设计的测试仿真系统，用于验证行为树功能的正确性。

## 系统架构

### 组件概述

1. **simulation_node** - 主仿真节点
    - 模拟飞机状态数据发布
    - 模拟目标检测数据发布
    - 提供各种ROS服务的模拟实现
    - 响应行为树的控制指令

2. **test_client** - 测试客户端
    - 发送任务JSON和命令
    - 交互式和自动化测试模式

3. **behavior_tree_tester.py** - 自动化测试脚本
    - 完整的测试流程管理
    - 测试结果评估和报告

4. **test_trees/** - 测试行为树XML文件
    - 简化的测试行为树定义
    - 覆盖主要功能模块

## 安装和编译

### 前置条件

- Ubuntu 20.04
- ROS2 Foxy
- behavior_node 包已编译
- custom_msgs 包已编译

### 编译步骤

```bash
# 创建工作空间（如果还没有）
mkdir -p ~/behavior_test_ws/src
cd ~/behavior_test_ws/src

# 将测试包代码放入src目录
# 复制所有文件到 behavior_test_simulation/ 目录

# 编译
cd ~/behavior_test_ws
colcon build --packages-select behavior_test_simulation

# 设置环境
source install/setup.bash