#!/bin/bash
# tools/run_all_tests.sh
# Behavior Tree 自动化测试运行脚本

set -e  # 遇到错误立即退出

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 配置变量
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"
PACKAGE_DIR="$(dirname "$SCRIPT_DIR")"
WORKSPACE_DIR="$(dirname "$(dirname "$PACKAGE_DIR")")"
RESULTS_DIR="$PACKAGE_DIR/results"
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
LOG_FILE="$RESULTS_DIR/test_run_$TIMESTAMP.log"

# 创建结果目录
mkdir -p "$RESULTS_DIR/test_reports"
mkdir -p "$RESULTS_DIR/performance_logs"
mkdir -p "$RESULTS_DIR/debug_traces"

echo -e "${BLUE}==================================================${NC}"
echo -e "${BLUE}    Behavior Tree 自动化测试系统${NC}"
echo -e "${BLUE}==================================================${NC}"
echo "时间戳: $TIMESTAMP"
echo "工作空间: $WORKSPACE_DIR"
echo "测试包: behavior_test_simulation"
echo "日志文件: $LOG_FILE"
echo ""

# 日志函数
log_info() {
    echo -e "${GREEN}[INFO]${NC} $1" | tee -a "$LOG_FILE"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1" | tee -a "$LOG_FILE"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1" | tee -a "$LOG_FILE"
}

# 检查依赖
check_dependencies() {
    log_info "检查系统依赖..."

    # 检查ROS2环境
    if [ -z "$ROS_DISTRO" ]; then
        log_error "ROS2环境未设置，请先source setup.bash"
        exit 1
    fi
    log_info "ROS2环境: $ROS_DISTRO"

    # 检查包是否编译
    if [ ! -f "$WORKSPACE_DIR/install/behavior_test_simulation/lib/behavior_test_simulation/simulation_node" ]; then
        log_error "behavior_test_simulation包未编译，请先运行colcon build"
        exit 1
    fi

    if [ ! -f "$WORKSPACE_DIR/install/behavior_node/lib/behavior_node/behavior_node" ]; then
        log_error "behavior_node包未编译，请先编译behavior_node"
        exit 1
    fi

    log_info "依赖检查通过"
}

# 启动仿真系统
start_simulation() {
    log_info "启动仿真系统..."

    # 启动仿真节点
    ros2 run behavior_test_simulation simulation_node \
        --ros-args \
        --params-file "$PACKAGE_DIR/config/simulation_params.yaml" \
        --log-level info &
    SIM_PID=$!

    # 启动behavior_node
    ros2 run behavior_node behavior_node \
        --ros-args \
        -p tree_directory:="$PACKAGE_DIR/test_trees" \
        -p tick_interval_ms:=50 \
        -p status_interval_ms:=1000 \
        --log-level info &
    BEHAVIOR_PID=$!

    # 等待节点启动
    sleep 5

    # 检查进程是否正常运行
    if ! kill -0 $SIM_PID 2>/dev/null; then
        log_error "仿真节点启动失败"
        cleanup_processes
        exit 1
    fi

    if ! kill -0 $BEHAVIOR_PID 2>/dev/null; then
        log_error "行为树节点启动失败"
        cleanup_processes
        exit 1
    fi

    log_info "仿真系统启动成功 (SIM_PID=$SIM_PID, BEHAVIOR_PID=$BEHAVIOR_PID)"
}

# 清理进程
cleanup_processes() {
    log_info "清理进程..."

    if [ ! -z "$SIM_PID" ] && kill -0 $SIM_PID 2>/dev/null; then
        kill $SIM_PID
        wait $SIM_PID 2>/dev/null || true
        log_info "仿真节点已停止"
    fi

    if [ ! -z "$BEHAVIOR_PID" ] && kill -0 $BEHAVIOR_PID 2>/dev/null; then
        kill $BEHAVIOR_PID
        wait $BEHAVIOR_PID 2>/dev/null || true
        log_info "行为树节点已停止"
    fi

    if [ ! -z "$TESTER_PID" ] && kill -0 $TESTER_PID 2>/dev/null; then
        kill $TESTER_PID
        wait $TESTER_PID 2>/dev/null || true
        log_info "测试脚本已停止"
    fi

    # 清理残留的ROS2进程
    pkill -f "simulation_node" 2>/dev/null || true
    pkill -f "behavior_node" 2>/dev/null || true
    pkill -f "behavior_tree_tester" 2>/dev/null || true
}

# 运行测试
run_tests() {
    log_info "开始运行测试..."

    local test_report="$RESULTS_DIR/test_reports/report_$TIMESTAMP.json"

    # 运行自动化测试脚本
    python3 "$PACKAGE_DIR/scripts/behavior_tree_tester.py" \
        --config "$PACKAGE_DIR/config/test_scenarios.yaml" \
        --output "$test_report" &
    TESTER_PID=$!

    # 等待测试完成
    wait $TESTER_PID
    local test_exit_code=$?

    if [ $test_exit_code -eq 0 ]; then
        log_info "测试完成，退出码: $test_exit_code"
    else
        log_error "测试失败，退出码: $test_exit_code"
    fi

    return $test_exit_code
}

# 生成测试报告
generate_report() {
    log_info "生成测试报告..."

    local report_file="$RESULTS_DIR/test_reports/summary_$TIMESTAMP.html"

    python3 "$PACKAGE_DIR/tools/generate_report.py" \
        --input-dir "$RESULTS_DIR/test_reports" \
        --output "$report_file" \
        --timestamp "$TIMESTAMP"

    if [ -f "$report_file" ]; then
        log_info "测试报告已生成: $report_file"

        # 如果有浏览器，自动打开报告
        if command -v firefox &> /dev/null; then
            firefox "$report_file" &
        elif command -v google-chrome &> /dev/null; then
            google-chrome "$report_file" &
        fi
    else
        log_warn "测试报告生成失败"
    fi
}

# 信号处理
trap cleanup_processes EXIT SIGINT SIGTERM

# 主函数
main() {
    local test_type="all"
    local quick_mode=false

    # 解析命令行参数
    while [[ $# -gt 0 ]]; do
        case $1 in
            --test-type)
                test_type="$2"
                shift 2
                ;;
            --quick)
                quick_mode=true
                shift
                ;;
            --help|-h)
                echo "用法: $0 [选项]"
                echo "选项:"
                echo "  --test-type TYPE    指定测试类型 (basic|complex|performance|stress|all)"
                echo "  --quick            快速模式，跳过长时间测试"
                echo "  --help, -h         显示帮助信息"
                exit 0
                ;;
            *)
                log_error "未知参数: $1"
                exit 1
                ;;
        esac
    done

    log_info "测试类型: $test_type"
    log_info "快速模式: $quick_mode"

    # 执行测试流程
    check_dependencies
    start_simulation

    if run_tests; then
        log_info "所有测试完成"
        generate_report

        echo ""
        echo -e "${GREEN}==================================================${NC}"
        echo -e "${GREEN}    测试运行完成！${NC}"
        echo -e "${GREEN}==================================================${NC}"
        echo "测试日志: $LOG_FILE"
        echo "测试报告目录: $RESULTS_DIR/test_reports/"

    else
        log_error "测试运行失败"
        echo ""
        echo -e "${RED}==================================================${NC}"
        echo -e "${RED}    测试运行失败！${NC}"
        echo -e "${RED}==================================================${NC}"
        echo "请检查日志文件: $LOG_FILE"
        exit 1
    fi
}

# 运行主函数
main "$@"