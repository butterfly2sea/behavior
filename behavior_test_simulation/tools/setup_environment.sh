#!/bin/bash
# tools/setup_environment.sh
# 设置Behavior Tree测试环境（支持混合依赖）

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}==================================================${NC}"
echo -e "${BLUE}    Behavior Tree 测试环境设置${NC}"
echo -e "${BLUE}==================================================${NC}"

# 获取脚本目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"
PACKAGE_DIR="$(dirname "$SCRIPT_DIR")"
WORKSPACE_DIR="$(dirname "$(dirname "$PACKAGE_DIR")")"

echo "工作空间路径: $WORKSPACE_DIR"
echo "测试包路径: $PACKAGE_DIR"

# 检查ROS2环境
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${RED}错误: ROS2环境未设置${NC}"
    echo "请先运行: source /opt/ros/foxy/setup.bash"
    exit 1
fi

echo -e "${GREEN}ROS2环境已设置: $ROS_DISTRO${NC}"

# 检查工作空间
if [ ! -f "$WORKSPACE_DIR/src/CMakeLists.txt" ] && [ ! -d "$WORKSPACE_DIR/src" ]; then
    echo -e "${YELLOW}警告: 当前不在ROS2工作空间中${NC}"
    echo "请确保在正确的工作空间目录中运行此脚本"
fi

# 设置工作空间环境
if [ -f "$WORKSPACE_DIR/install/setup.bash" ]; then
    echo "设置工作空间环境..."
    source "$WORKSPACE_DIR/install/setup.bash"
    echo -e "${GREEN}工作空间环境已设置${NC}"
else
    echo -e "${YELLOW}警告: 工作空间未编译，请先运行 colcon build${NC}"
fi

# 检查系统安装的custom_msgs
echo ""
echo "检查系统依赖..."

check_system_custom_msgs() {
    echo -n "检查 custom_msgs (系统安装): "

    # 检查头文件
    if [ -d "/usr/local/include/custom_msgs" ]; then
        echo -e "${GREEN}✓ 头文件存在${NC}"

        # 列出找到的消息类型
        local msg_count=$(find /usr/local/include/custom_msgs -name "*.hpp" -o -name "*.h" | wc -l)
        echo "  发现 $msg_count 个消息/服务定义文件"

        # 检查关键文件
        local key_files=(
            "/usr/local/include/custom_msgs/msg/simple_vehicle.hpp"
            "/usr/local/include/custom_msgs/msg/object_computation.hpp"
            "/usr/local/include/custom_msgs/msg/offboard_ctrl.hpp"
            "/usr/local/include/custom_msgs/srv/command_bool.hpp"
            "/usr/local/include/custom_msgs/srv/command_long.hpp"
        )

        local missing_files=0
        for file in "${key_files[@]}"; do
            if [ ! -f "$file" ]; then
                echo -e "  ${YELLOW}⚠ 缺少: $(basename "$file")${NC}"
                ((missing_files++))
            fi
        done

        if [ $missing_files -eq 0 ]; then
            echo -e "  ${GREEN}✓ 所有关键消息类型都存在${NC}"
        else
            echo -e "  ${YELLOW}⚠ 缺少 $missing_files 个关键文件${NC}"
        fi

        return 0
    else
        echo -e "${RED}✗ 未找到${NC}"
        echo "  请确保 custom_msgs 已正确安装到 /usr/local/include/"
        return 1
    fi
}

if ! check_system_custom_msgs; then
    echo -e "${RED}错误: custom_msgs 未正确安装${NC}"
    echo ""
    echo "安装建议："
    echo "1. 重新编译并安装 custom_msgs:"
    echo "   cd /path/to/custom_msgs"
    echo "   mkdir build && cd build"
    echo "   cmake .."
    echo "   make"
    echo "   sudo make install"
    echo ""
    echo "2. 或者检查安装路径是否正确"
    exit 1
fi

# 检查ROS2包依赖
echo ""
echo "检查ROS2包依赖..."

check_ros_package() {
    local pkg_name=$1
    if ros2 pkg list | grep -q "^$pkg_name$"; then
        echo -e "${GREEN}✓${NC} $pkg_name"
        return 0
    else
        echo -e "${RED}✗${NC} $pkg_name (缺失)"
        return 1
    fi
}

missing_packages=0

# 检查核心ROS2包
ros_packages=(
    "rclcpp"
    "std_msgs"
    "sensor_msgs"
    "geometry_msgs"
    "mavros_msgs"
)

for pkg in "${ros_packages[@]}"; do
    if ! check_ros_package "$pkg"; then
        ((missing_packages++))
    fi
done

# 检查可选的behavior_node包
if check_ros_package "behavior_node"; then
    echo -e "  ${GREEN}✓ behavior_node 包可用${NC}"
else
    echo -e "  ${YELLOW}⚠ behavior_node 包未编译${NC}"
    echo "  请先编译 behavior_node 包"
fi

if [ $missing_packages -gt 0 ]; then
    echo -e "${RED}错误: 缺少 $missing_packages 个ROS2包${NC}"
    echo "请安装缺失的包："
    echo "sudo apt update"
    echo "sudo apt install ros-foxy-rclcpp ros-foxy-std-msgs ros-foxy-sensor-msgs ros-foxy-geometry-msgs ros-foxy-mavros-msgs"
    exit 1
fi

# 检查Python依赖
echo ""
echo "检查Python依赖..."

check_python_package() {
    local pkg_name=$1
    local import_name=${2:-$pkg_name}
    if python3 -c "import $import_name" 2>/dev/null; then
        echo -e "${GREEN}✓${NC} $pkg_name"
        return 0
    else
        echo -e "${RED}✗${NC} $pkg_name (缺失)"
        return 1
    fi
}

missing_python_packages=0

python_packages=(
    "yaml:yaml"
    "json:json"
    "numpy:numpy"
    "nlohmann_json (可选):json"
)

for pkg_info in "${python_packages[@]}"; do
    IFS=':' read -r pkg_name import_name <<< "$pkg_info"
    if ! check_python_package "$pkg_name" "$import_name"; then
        if [[ "$pkg_name" != *"可选"* ]]; then
            ((missing_python_packages++))
        fi
    fi
done

if [ $missing_python_packages -gt 0 ]; then
    echo -e "${YELLOW}警告: 缺少 $missing_python_packages 个Python包${NC}"
    echo "安装命令："
    echo "pip3 install PyYAML numpy"
fi

# 设置环境变量
echo ""
echo "设置环境变量..."

# 确保custom_msgs头文件路径在环境中
export CPLUS_INCLUDE_PATH="/usr/local/include:${CPLUS_INCLUDE_PATH:-}"
export C_INCLUDE_PATH="/usr/local/include:${C_INCLUDE_PATH:-}"
export LIBRARY_PATH="/usr/local/lib:${LIBRARY_PATH:-}"
export LD_LIBRARY_PATH="/usr/local/lib:${LD_LIBRARY_PATH:-}"

echo -e "${GREEN}✓ 环境变量已设置${NC}"

# 创建必要的目录
echo ""
echo "创建目录结构..."

directories=(
    "$PACKAGE_DIR/results"
    "$PACKAGE_DIR/results/test_reports"
    "$PACKAGE_DIR/results/performance_logs"
    "$PACKAGE_DIR/results/debug_traces"
)

for dir in "${directories[@]}"; do
    if [ ! -d "$dir" ]; then
        mkdir -p "$dir"
        echo -e "${GREEN}创建目录:${NC} $dir"
    else
        echo -e "${BLUE}目录已存在:${NC} $dir"
    fi
done

# 设置脚本权限
echo ""
echo "设置脚本权限..."

scripts=(
    "$PACKAGE_DIR/tools/run_all_tests.sh"
    "$PACKAGE_DIR/tools/clean_workspace.sh"
    "$PACKAGE_DIR/scripts/behavior_tree_tester.py"
)

for script in "${scripts[@]}"; do
    if [ -f "$script" ]; then
        chmod +x "$script"
        echo -e "${GREEN}设置执行权限:${NC} $(basename "$script")"
    else
        echo -e "${YELLOW}脚本不存在:${NC} $(basename "$script")"
    fi
done

# 验证配置文件
echo ""
echo "验证配置文件..."

config_files=(
    "$PACKAGE_DIR/config/simulation_params.yaml"
    "$PACKAGE_DIR/config/test_scenarios.yaml"
)

for config in "${config_files[@]}"; do
    if [ -f "$config" ]; then
        echo -e "${GREEN}✓${NC} $(basename "$config")"
    else
        echo -e "${RED}✗${NC} $(basename "$config") (缺失)"
    fi
done

# 验证测试行为树文件
echo ""
echo "验证测试行为树..."

if [ -d "$PACKAGE_DIR/test_trees" ]; then
    tree_count=$(find "$PACKAGE_DIR/test_trees" -name "*.xml" | wc -l)
    echo -e "${GREEN}发现 $tree_count 个行为树文件${NC}"

    if [ $tree_count -eq 0 ]; then
        echo -e "${YELLOW}警告: test_trees目录中没有XML文件${NC}"
    fi
else
    echo -e "${RED}错误: test_trees目录不存在${NC}"
fi

# 创建编译辅助脚本
echo ""
echo "创建编译辅助脚本..."

cat > "$PACKAGE_DIR/build_with_custom_msgs.sh" << 'EOF'
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
EOF

chmod +x "$PACKAGE_DIR/build_with_custom_msgs.sh"
echo -e "${GREEN}创建编译脚本: build_with_custom_msgs.sh${NC}"

# 生成启动快捷脚本
echo ""
echo "生成启动快捷脚本..."

cat > "$PACKAGE_DIR/start_test.sh" << 'EOF'
#!/bin/bash
# 快速启动测试脚本

cd "$(dirname "$0")"
source tools/setup_environment.sh
exec tools/run_all_tests.sh "$@"
EOF

chmod +x "$PACKAGE_DIR/start_test.sh"
echo -e "${GREEN}创建快捷启动脚本: start_test.sh${NC}"

echo ""
echo -e "${GREEN}==================================================${NC}"
echo -e "${GREEN}    环境设置完成！${NC}"
echo -e "${GREEN}==================================================${NC}"
echo ""
echo "使用说明："
echo "1. 编译测试包:"
echo "   ./build_with_custom_msgs.sh"
echo ""
echo "2. 运行测试:"
echo "   ./start_test.sh              - 快速启动所有测试"
echo "   ./tools/run_all_tests.sh     - 运行完整测试套件"
echo ""
echo "3. 清理环境:"
echo "   ./tools/clean_workspace.sh   - 清理工作空间"
echo ""
echo "重要提示："
echo "- custom_msgs 通过系统安装 (/usr/local/include/)"
echo "- 使用专用编译脚本处理混合依赖"
echo "- 确保在每次编译前都设置了正确的环境变量"
echo ""

if [ $missing_packages -eq 0 ] && [ $missing_python_packages -eq 0 ]; then
    echo -e "${GREEN}✓ 环境设置完整，可以开始编译和测试！${NC}"
    echo ""
    echo "下一步："
    echo "1. 运行: ./build_with_custom_msgs.sh"
    echo "2. 然后: source $WORKSPACE_DIR/install/setup.bash"
    echo "3. 最后: ./start_test.sh"
    exit 0
else
    echo -e "${YELLOW}⚠ 环境设置完成，但存在缺失依赖${NC}"
    echo "请解决上述依赖问题后再运行编译"
    exit 1
fi