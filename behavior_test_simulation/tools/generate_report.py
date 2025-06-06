#!/usr/bin/env python3
# tools/generate_report.py
# 测试报告生成器

import json
import os
import sys
import argparse
from datetime import datetime
from pathlib import Path

def generate_html_report(test_data, output_file, timestamp):
    """生成HTML格式的测试报告"""

    # 计算统计信息
    total_tests = len(test_data)
    passed_tests = sum(1 for test in test_data.values() if test.get('success', False))
    failed_tests = total_tests - passed_tests
    success_rate = (passed_tests / total_tests * 100) if total_tests > 0 else 0

    # HTML模板
    html_template = f"""
<!DOCTYPE html>
<html lang="zh-CN">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Behavior Tree 测试报告 - {timestamp}</title>
    <style>
        body {{
            font-family: 'Arial', sans-serif;
            margin: 0;
            padding: 20px;
            background-color: #f5f5f5;
        }}
        .container {{
            max-width: 1200px;
            margin: 0 auto;
            background-color: white;
            padding: 30px;
            border-radius: 10px;
            box-shadow: 0 0 20px rgba(0,0,0,0.1);
        }}
        .header {{
            text-align: center;
            border-bottom: 2px solid #eee;
            padding-bottom: 20px;
            margin-bottom: 30px;
        }}
        .title {{
            color: #2c3e50;
            font-size: 28px;
            margin: 0;
        }}
        .timestamp {{
            color: #7f8c8d;
            font-size: 14px;
            margin-top: 10px;
        }}
        .summary {{
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
            gap: 20px;
            margin-bottom: 30px;
        }}
        .summary-card {{
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            color: white;
            padding: 20px;
            border-radius: 8px;
            text-align: center;
        }}
        .summary-card.success {{
            background: linear-gradient(135deg, #11998e 0%, #38ef7d 100%);
        }}
        .summary-card.failure {{
            background: linear-gradient(135deg, #ff512f 0%, #f09819 100%);
        }}
        .summary-card.rate {{
            background: linear-gradient(135deg, #4facfe 0%, #00f2fe 100%);
        }}
        .summary-number {{
            font-size: 36px;
            font-weight: bold;
            display: block;
        }}
        .summary-label {{
            font-size: 14px;
            opacity: 0.9;
        }}
        .test-results {{
            margin-top: 30px;
        }}
        .section-title {{
            color: #2c3e50;
            font-size: 20px;
            margin-bottom: 20px;
            border-left: 4px solid #3498db;
            padding-left: 15px;
        }}
        .test-item {{
            background: white;
            border: 1px solid #e1e8ed;
            border-radius: 8px;
            margin-bottom: 15px;
            overflow: hidden;
        }}
        .test-header {{
            padding: 15px 20px;
            display: flex;
            justify-content: space-between;
            align-items: center;
            cursor: pointer;
            transition: background-color 0.3s;
        }}
        .test-header:hover {{
            background-color: #f8f9fa;
        }}
        .test-name {{
            font-weight: bold;
            color: #2c3e50;
        }}
        .test-status {{
            padding: 4px 12px;
            border-radius: 20px;
            font-size: 12px;
            font-weight: bold;
            text-transform: uppercase;
        }}
        .test-status.passed {{
            background-color: #d4edda;
            color: #155724;
        }}
        .test-status.failed {{
            background-color: #f8d7da;
            color: #721c24;
        }}
        .test-details {{
            padding: 0 20px 20px;
            background-color: #f8f9fa;
            border-top: 1px solid #e1e8ed;
            display: none;
        }}
        .test-details.show {{
            display: block;
        }}
        .detail-row {{
            display: flex;
            justify-content: space-between;
            margin-bottom: 8px;
        }}
        .detail-label {{
            font-weight: bold;
            color: #6c757d;
        }}
        .detail-value {{
            color: #495057;
        }}
        .footer {{
            margin-top: 40px;
            text-align: center;
            color: #6c757d;
            font-size: 12px;
            border-top: 1px solid #eee;
            padding-top: 20px;
        }}
        .progress-bar {{
            width: 100%;
            height: 8px;
            background-color: #e9ecef;
            border-radius: 4px;
            overflow: hidden;
            margin-top: 10px;
        }}
        .progress-fill {{
            height: 100%;
            background: linear-gradient(90deg, #11998e 0%, #38ef7d 100%);
            transition: width 0.3s ease;
        }}
    </style>
</head>
<body>
    <div class="container">
        <div class="header">
            <h1 class="title">Behavior Tree 测试报告</h1>
            <div class="timestamp">生成时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}</div>
        </div>
        
        <div class="summary">
            <div class="summary-card">
                <span class="summary-number">{total_tests}</span>
                <span class="summary-label">总测试数</span>
            </div>
            <div class="summary-card success">
                <span class="summary-number">{passed_tests}</span>
                <span class="summary-label">通过测试</span>
            </div>
            <div class="summary-card failure">
                <span class="summary-number">{failed_tests}</span>
                <span class="summary-label">失败测试</span>
            </div>
            <div class="summary-card rate">
                <span class="summary-number">{success_rate:.1f}%</span>
                <span class="summary-label">成功率</span>
            </div>
        </div>
        
        <div class="progress-bar">
            <div class="progress-fill" style="width: {success_rate}%"></div>
        </div>
        
        <div class="test-results">
            <h2 class="section-title">详细测试结果</h2>
"""

    # 添加每个测试项的详细信息
    for test_name, test_result in test_data.items():
        success = test_result.get('success', False)
        status_class = 'passed' if success else 'failed'
        status_text = '通过' if success else '失败'
        duration = test_result.get('duration', 0)
        message = test_result.get('message', '无信息')

        html_template += f"""
            <div class="test-item">
                <div class="test-header" onclick="toggleDetails('{test_name}')">
                    <span class="test-name">{test_name}</span>
                    <span class="test-status {status_class}">{status_text}</span>
                </div>
                <div class="test-details" id="details-{test_name}">
                    <div class="detail-row">
                        <span class="detail-label">执行时间:</span>
                        <span class="detail-value">{duration:.2f} 秒</span>
                    </div>
                    <div class="detail-row">
                        <span class="detail-label">结果消息:</span>
                        <span class="detail-value">{message}</span>
                    </div>
                    <div class="detail-row">
                        <span class="detail-label">状态:</span>
                        <span class="detail-value">{'执行成功' if success else '执行失败'}</span>
                    </div>
                </div>
            </div>
"""

    # 完成HTML
    html_template += f"""
        </div>
        
        <div class="footer">
            <p>报告由 Behavior Tree 测试系统自动生成</p>
            <p>测试时间戳: {timestamp}</p>
        </div>
    </div>
    
    <script>
        function toggleDetails(testName) {{
            const details = document.getElementById('details-' + testName);
            details.classList.toggle('show');
        }}
        
        // 自动展开失败的测试
        document.addEventListener('DOMContentLoaded', function() {{
            const failedTests = document.querySelectorAll('.test-status.failed');
            failedTests.forEach(function(element) {{
                const testItem = element.closest('.test-item');
                const testHeader = testItem.querySelector('.test-header');
                testHeader.click();
            }});
        }});
    </script>
</body>
</html>
"""

    # 写入文件
    with open(output_file, 'w', encoding='utf-8') as f:
        f.write(html_template)

def load_test_results(input_dir):
    """加载测试结果数据"""
    test_data = {}

    # 查找所有JSON报告文件
    input_path = Path(input_dir)
    if input_path.is_file() and input_path.suffix == '.json':
        # 单个文件
        json_files = [input_path]
    else:
        # 目录中的所有JSON文件
        json_files = list(input_path.glob('*.json'))

    for json_file in json_files:
        try:
            with open(json_file, 'r', encoding='utf-8') as f:
                data = json.load(f)
                if isinstance(data, dict):
                    test_data.update(data)
                else:
                    print(f"警告: {json_file} 不是有效的测试数据格式")
        except json.JSONDecodeError as e:
            print(f"错误: 无法解析 {json_file}: {e}")
        except FileNotFoundError:
            print(f"错误: 文件不存在 {json_file}")

    return test_data

def main():
    parser = argparse.ArgumentParser(description='生成Behavior Tree测试报告')
    parser.add_argument('--input-dir', required=True, help='测试结果输入目录或文件')
    parser.add_argument('--output', required=True, help='输出HTML文件路径')
    parser.add_argument('--timestamp', default='', help='时间戳')

    args = parser.parse_args()

    # 加载测试数据
    test_data = load_test_results(args.input_dir)

    if not test_data:
        print("错误: 没有找到有效的测试数据")
        sys.exit(1)

    # 生成时间戳
    timestamp = args.timestamp if args.timestamp else datetime.now().strftime('%Y%m%d_%H%M%S')

    # 生成报告
    try:
        generate_html_report(test_data, args.output, timestamp)
        print(f"测试报告已生成: {args.output}")
        print(f"总测试数: {len(test_data)}")
        print(f"通过测试: {sum(1 for t in test_data.values() if t.get('success', False))}")

    except Exception as e:
        print(f"错误: 生成报告失败: {e}")
        sys.exit(1)

if __name__ == '__main__':
    main()