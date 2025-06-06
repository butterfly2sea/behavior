#!/bin/bash
# tools/clean_workspace.sh
# 清理测试工作空间

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# 获取脚本目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"
PACKAGE_DIR="$(dirname "$SCRIPT_DIR")"

echo -e "${BLUE}==================================================${NC}"
echo -e "${BLUE}    清理 Behavior Tree 测试工作空间${NC}"
echo -e "${BLUE}==================================================${NC}"

# 解析命令行参数
CLEAN_ALL=false
CLEAN_LOGS=false
CLEAN_REPORTS=false
CLEAN_PROCESSES=false
INTERACTIVE=true

while [[ $# -gt 0 ]]; do
    case $1 in
        --all)
            CLEAN_ALL=true
            INTERACTIVE=false
            shift
            ;;
        --logs)
            CLEAN_LOGS=true
            INTERACTIVE=false
            shift
            ;;
        --reports)
            CLEAN_REPORTS=true
            INTERACTIVE=false
            shift
            ;;
        --processes)
            CLEAN_PROCESSES=true
            INTERACTIVE=false
            shift
            ;;
        --yes|-y)
            INTERACTIVE=false
            shift
            ;;
        --help|-h)
            echo "用法: $0 [选项]"
            echo "选项:"
            echo "  --all        清理所有内容"
            echo "  --logs       只清理日志文件"
            echo "  --reports    只清理测试报告"
            echo "  --processes  只终止相关进程"
            echo "  --yes, -y    跳过确认提示"
            echo "  --help, -h   显示帮助信息"
            exit 0
            ;;
        *)
            echo -e "${RED}错误: 未知参数 $1${NC}"
            exit 1
            ;;
    esac
done

# 确认函数
confirm() {
    if [ "$INTERACTIVE" = true ]; then
        echo -n -e "${YELLOW}$1 (y/N): ${NC}"
        read -r response
        case "$response" in
            [yY][eE][sS]|[yY])
                return 0
                ;;
            *)
                return 1
                ;;
        esac
    else
        return 0
    fi
}

# 终止相关进程
cleanup_processes() {
    echo "正在终止相关进程..."

    # 查找并终止相关进程
    local processes=(
        "simulation_node"
        "behavior_node"
        "behavior_tree_tester"
        "test_client"
    )

    for process in "${processes[@]}"; do
        local pids=$(pgrep -f "$process" 2>/dev/null || true)
        if [ -n "$pids" ]; then
            echo -e "${YELLOW}终止进程: $process (PIDs: $pids)${NC}"
            kill $pids 2>/dev/null || true
            sleep 1

            # 强制终止仍在运行的进程
            local remaining_pids=$(pgrep -f "$process" 2>/dev/null || true)
            if [ -n "$remaining_pids" ]; then
                echo -e "${RED}强制终止进程: $process (PIDs: $remaining_pids)${NC}"
                kill -9 $remaining_pids 2>/dev/null || true
            fi
        fi
    done

    echo -e "${GREEN}✓ 进程清理完成${NC}"
}

# 清理日志文件
cleanup_logs() {
    echo "正在清理日志文件..."

    local log_dirs=(
        "$PACKAGE_DIR/results/performance_logs"
        "$PACKAGE_DIR/results/debug_traces"
        "$HOME/.ros/log"
    )

    for log_dir in "${log_dirs[@]}"; do
        if [ -d "$log_dir" ]; then
            local file_count=$(find "$log_dir" -type f | wc -l)
            if [ $file_count -gt 0 ]; then
                echo -e "${YELLOW}清理目录: $log_dir ($file_count 个文件)${NC}"
                find "$log_dir" -type f -delete
            fi
        fi
    done

    # 清理根目录下的日志文件
    if [ -d "$PACKAGE_DIR/results" ]; then
        find "$PACKAGE_DIR/results" -name "*.log" -delete 2>/dev/null || true
    fi

    echo -e "${GREEN}✓ 日志清理完成${NC}"
}

# 清理测试报告
cleanup_reports() {
    echo "正在清理测试报告..."

    local report_dir="$PACKAGE_DIR/results/test_reports"

    if [ -d "$report_dir" ]; then
        local file_count=$(find "$report_dir" -type f | wc -l)
        if [ $file_count -gt 0 ]; then
            echo -e "${YELLOW}清理目录: $report_dir ($file_count 个文件)${NC}"
            find "$report_dir" -type f -delete
        fi
    fi

    echo -e "${GREEN}✓ 测试报告清理完成${NC}"
}

# 清理临时文件
cleanup_temp_files() {
    echo "正在清理临时文件..."

    # 清理Python缓存
    find "$PACKAGE_DIR" -name "__pycache__" -type d -exec rm -rf {} + 2>/dev/null || true
    find "$PACKAGE_DIR" -name "*.pyc" -delete 2>/dev/null || true

    # 清理编辑器临时文件
    find "$PACKAGE_DIR" -name "*~" -delete 2>/dev/null || true
    find "$PACKAGE_DIR" -name "*.swp" -delete 2>/dev/null || true
    find "$PACKAGE_DIR" -name ".*.swp" -delete 2>/dev/null || true

    # 清理其他临时文件
    find "$PACKAGE_DIR" -name "core.*" -delete 2>/dev/null || true

    echo -e "${GREEN}✓ 临时文件清理完成${NC}"
}

# 显示磁盘使用情况
show_disk_usage() {
    echo ""
    echo "清理前后磁盘使用情况:"

    if [ -d "$PACKAGE_DIR/results" ]; then
        local size=$(du -sh "$PACKAGE_DIR/results" 2>/dev/null | cut -f1)
        echo "results目录大小: $size"
    fi

    local workspace_size=$(du -sh "$PACKAGE_DIR" 2>/dev/null | cut -f1)
    echo "测试包总大小: $workspace_size"
}

# 主清理逻辑
main() {
    echo "测试包路径: $PACKAGE_DIR"
    echo ""

    # 显示清理前的磁盘使用情况
    show_disk_usage
    echo ""

    # 根据参数决定清理内容
    if [ "$CLEAN_ALL" = true ]; then
        if confirm "确定要清理所有内容吗？这将删除所有日志、报告和临时文件"; then
            cleanup_processes
            cleanup_logs
            cleanup_reports
            cleanup_temp_files
        fi
    elif [ "$CLEAN_PROCESSES" = true ]; then
        cleanup_processes
    elif [ "$CLEAN_LOGS" = true ]; then
        if confirm "确定要清理所有日志文件吗？"; then
            cleanup_logs
        fi
    elif [ "$CLEAN_REPORTS" = true ]; then
        if confirm "确定要清理所有测试报告吗？"; then
            cleanup_reports
        fi
    else
        # 交互式模式
        echo "请选择要清理的内容:"
        echo ""

        if confirm "1. 终止相关进程"; then
            cleanup_processes
        fi

        if confirm "2. 清理日志文件"; then
            cleanup_logs
        fi

        if confirm "3. 清理测试报告"; then
            cleanup_reports
        fi

        if confirm "4. 清理临时文件"; then
            cleanup_temp_files
        fi
    fi

    echo ""
    echo -e "${GREEN}==================================================${NC}"
    echo -e "${GREEN}    清理完成！${NC}"
    echo -e "${GREEN}==================================================${NC}"

    # 显示清理后的磁盘使用情况
    show_disk_usage
}

# 运行主函数
main "$@"