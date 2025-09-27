#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
自主泊车评分器 - Autonomous Parking Scoring

用于评估自主泊车竞赛规则：
1. 超时90秒：直接0分
2. 进入禁行区：直接0分
3. 停车压线：扣20分
4. 成功泊车：100分

作者: Apollo竞赛团队
版本: 1.0
日期: 2025-09-27
"""

import sys
import json
import math
from typing import List, Dict, Any, Optional


class ParkingScorer:
    """自主泊车评分器"""

    def __init__(self, time_limit_sec: float = 90.0, line_violation_penalty: int = 20):
        """
        初始化评分器

        Args:
            time_limit_sec: 时间限制(秒)
            line_violation_penalty: 压线违规扣分
        """
        self.time_limit_sec = time_limit_sec
        self.line_violation_penalty = line_violation_penalty

    def score_parking_scenario(self, log_data: Dict[str, Any]) -> Dict[str, Any]:
        """
        评估泊车场景得分

        Args:
            log_data: 包含场景执行日志的字典

        Returns:
            评分结果字典
        """
        violations = []
        score = 100

        # 检查超时
        scenario_duration = log_data.get('scenario_duration', 0.0)
        if scenario_duration > self.time_limit_sec:
            violations.append({
                'type': 'TIMEOUT',
                'message': f'Scenario exceeded time limit: {scenario_duration:.1f}s > {self.time_limit_sec}s',
                'penalty': 100
            })
            score = 0

        # 检查禁行区进入
        forbidden_zone_entries = log_data.get('forbidden_zone_entries', [])
        if forbidden_zone_entries:
            violations.append({
                'type': 'FORBIDDEN_ZONE_ENTRY',
                'message': f'Vehicle entered forbidden zone {len(forbidden_zone_entries)} times',
                'details': forbidden_zone_entries,
                'penalty': 100
            })
            score = 0

        # 检查停车压线违规（仅在成功完成且未超时/禁区违规时检查）
        parking_line_violation = log_data.get('parking_line_violation', False)
        if score > 0 and parking_line_violation:
            violations.append({
                'type': 'PARKING_LINE_VIOLATION',
                'message': 'Parking line violation detected',
                'penalty': self.line_violation_penalty
            })
            score = max(0, score - self.line_violation_penalty)

        # 检查泊车是否成功完成
        scenario_success = log_data.get('scenario_success', False)
        if not scenario_success and score > 0:
            # 如果场景未成功完成且没有其他违规，可能是规划失败
            violations.append({
                'type': 'PLANNING_FAILURE',
                'message': 'Parking scenario failed to complete successfully',
                'penalty': 100
            })
            score = 0

        return {
            'scenario': 'autonomous_parking',
            'scenario_duration': scenario_duration,
            'scenario_success': scenario_success,
            'time_limit_exceeded': scenario_duration > self.time_limit_sec,
            'forbidden_zone_violation': len(forbidden_zone_entries) > 0,
            'parking_line_violation': parking_line_violation,
            'final_score': score,
            'violations': violations,
            'violation_count': len(violations)
        }

    def parse_apollo_logs(self, log_file: str) -> Dict[str, Any]:
        """
        从Apollo日志文件中解析泊车场景数据

        Args:
            log_file: 日志文件路径

        Returns:
            解析后的场景数据
        """
        log_data = {
            'scenario_duration': 0.0,
            'scenario_success': False,
            'forbidden_zone_entries': [],
            'parking_line_violation': False
        }

        try:
            with open(log_file, 'r', encoding='utf-8') as f:
                for line in f:
                    line = line.strip()

                    # 解析场景开始时间
                    if '[COMPETITION_PARKING] Selected nearest parking spot' in line:
                        # 记录场景开始（这里简化，实际应从时间戳解析）
                        log_data['scenario_started'] = True

                    # 解析超时违规
                    if '[COMPETITION_PARKING_TIMEOUT]' in line:
                        # 提取超时时间
                        log_data['scenario_duration'] = 95.0  # 示例，实际应从日志解析

                    # 解析禁行区违规
                    if '[COMPETITION_FORBIDDEN_ZONE]' in line:
                        forbidden_entry = self._parse_forbidden_zone_entry(line)
                        if forbidden_entry:
                            log_data['forbidden_zone_entries'].append(forbidden_entry)

                    # 解析压线违规
                    if '[COMPETITION_PARKING_LINE_VIOLATION]' in line:
                        log_data['parking_line_violation'] = True

                    # 解析场景成功完成
                    if 'StageParking finished' in line or 'parking completed successfully' in line.lower():
                        log_data['scenario_success'] = True

        except Exception as e:
            print(f"Error parsing log file: {e}")

        return log_data

    def _parse_forbidden_zone_entry(self, log_line: str) -> Optional[Dict[str, Any]]:
        """
        从日志行解析禁行区进入信息

        Args:
            log_line: 日志行

        Returns:
            禁行区进入信息或None
        """
        # 简化解析，实际应根据日志格式调整
        try:
            # 示例：解析坐标信息
            if '(' in log_line and ')' in log_line:
                coord_start = log_line.find('(')
                coord_end = log_line.find(')')
                if coord_start != -1 and coord_end != -1:
                    coord_str = log_line[coord_start+1:coord_end]
                    coords = coord_str.split(',')
                    if len(coords) >= 2:
                        return {
                            'x': float(coords[0].strip()),
                            'y': float(coords[1].strip()),
                            'timestamp': 'unknown'  # 实际应从日志时间戳解析
                        }
        except:
            pass

        return {'x': 0.0, 'y': 0.0, 'timestamp': 'unknown'}

    def print_results(self, results: Dict[str, Any]) -> None:
        """打印评分结果"""
        print("\n" + "="*60)
        print("🚗 自主泊车评分结果")
        print("="*60)

        print(f"⏱️  场景时长: {results['scenario_duration']:.1f}秒")
        print(f"📊 得分: {results['final_score']}/100")

        if results['scenario_success'] and results['final_score'] == 100:
            print("✅ 结果: 完美泊车")
        elif results['scenario_success'] and results['final_score'] > 0:
            print("⚠️  结果: 泊车成功但有扣分")
        else:
            print("❌ 结果: 泊车失败")

        if results['violations']:
            print(f"\n🚫 违规记录 ({len(results['violations'])}项):")
            for violation in results['violations'][:10]:  # 只显示前10项
                print(f"  • {violation['type']}: {violation['message']}")
                print(f"    💰 扣分: -{violation['penalty']}")

            if len(results['violations']) > 10:
                print(f"  ... 还有 {len(results['violations']) - 10} 项违规")

        print(f"\n💰 总扣分: -{100 - results['final_score']}")
        print(f"🏆 最终得分: {results['final_score']}")


def main():
    """主函数"""
    import argparse

    parser = argparse.ArgumentParser(description='Apollo自主泊车评分器')
    parser.add_argument('--log-file', required=True,
                       help='Apollo规划日志文件')
    parser.add_argument('--time-limit', type=float, default=90.0,
                       help='时间限制(秒)')
    parser.add_argument('--line-penalty', type=int, default=20,
                       help='压线违规扣分')
    parser.add_argument('--output', type=str, default=None,
                       help='结果输出文件 (JSON格式)')

    args = parser.parse_args()

    # 创建评分器
    scorer = ParkingScorer(args.time_limit, args.line_penalty)

    # 解析日志
    log_data = scorer.parse_apollo_logs(args.log_file)

    # 评分
    results = scorer.score_parking_scenario(log_data)

    # 打印结果
    scorer.print_results(results)

    # 保存结果
    if args.output:
        with open(args.output, 'w', encoding='utf-8') as f:
            json.dump(results, f, indent=2, ensure_ascii=False)
        print(f"\n结果已保存到: {args.output}")

    # 返回退出码
    sys.exit(0 if results['final_score'] > 0 else 1)


if __name__ == '__main__':
    main()
