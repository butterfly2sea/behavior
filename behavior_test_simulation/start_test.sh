#!/bin/bash
# 快速启动测试脚本

cd "$(dirname "$0")"
source tools/setup_environment.sh
exec tools/run_all_tests.sh "$@"
