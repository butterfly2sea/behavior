#!/bin/bash
# 编译脚本，处理custom_msgs依赖

cd "$(dirname "$0")"
PACKAGE_DIR="$(pwd)"
WORKSPACE_DIR="$(dirname "$(dirname "$PACKAGE_DIR")")"

echo "工作空间: $WORKSPACE_DIR"
echo "包目录: $PACKAGE_DIR"

# 设置环境变量
export CPLUS_INCLUDE_PATH="/usr/local/include:${CPLUS_INCLUDE_PATH:-}"
export C_INCLUDE_PATH="/usr/local/include:${C_INCLUDE_PATH:-}"
export LIBRARY_PATH="/usr/local/lib:${LIBRARY_PATH:-}"
export LD_LIBRARY_PATH="/usr/local/lib:${LD_LIBRARY_PATH:-}"

# 进入工作空间并编译
cd "$WORKSPACE_DIR"

echo "开始编译 behavior_test_simulation..."
colcon build --packages-select behavior_test_simulation --cmake-args -DCMAKE_BUILD_TYPE=Release

if [ $? -eq 0 ]; then
    echo "编译成功！"
    echo "请运行: source install/setup.bash"
else
    echo "编译失败，请检查错误信息"
fi
