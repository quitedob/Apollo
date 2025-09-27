#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
施工区评分器 - Construction Zone Scoring

用于评估施工区域通行竞赛规则：
1. 驶入施工区域：直接得0分
2. 施工区域内超速：每超速1m/s每帧扣2分

作者: Apollo竞赛团队
版本: 1.0
日期: 2025-09-27
"""

import sys
import json
import math
from typing import List, Dict, Any, Tuple, Optional


class ConstructionZoneScorer:
    """施工区评分器"""

    def __init__(self, speed_limit: float = 8.333, penalty_per_frame_per_mps: float = 2.0):
        """
        初始化评分器

        Args:
            speed_limit: 施工区限速 (m/s)
            penalty_per_frame_per_mps: 每帧每超速1m/s的扣分
        """
        self.speed_limit = speed_limit
        self.penalty_per_frame_per_mps = penalty_per_frame_per_mps

    def score_trajectory(self, trajectory_frames: List[Dict[str, Any]],
                        construction_zones: List[Dict[str, Any]]) -> Dict[str, Any]:
        """
        评估轨迹得分

        Args:
            trajectory_frames: 轨迹帧列表，每个帧包含x, y, v, timestamp等
            construction_zones: 施工区定义列表

        Returns:
            评分结果字典
        """
        violations = []
        total_penalty = 0
        entered_construction = False

        # 检查每一帧
        for frame_idx, frame in enumerate(trajectory_frames):
            x = frame.get('x', 0.0)
            y = frame.get('y', 0.0)
            v = frame.get('v', 0.0)
            timestamp = frame.get('timestamp', frame_idx * 0.1)  # 默认10Hz

            # 检查是否进入施工区
            in_construction_zone = False
            for zone in construction_zones:
                if self._point_in_zone(x, y, zone):
                    in_construction_zone = True

                    # 记录进入施工区违规（直接0分）
                    if not entered_construction:
                        entered_construction = True
                        violations.append({
                            'type': 'CONSTRUCTION_ZONE_ENTRY',
                            'frame': frame_idx,
                            'timestamp': timestamp,
                            'x': x,
                            'y': y,
                            'message': 'Vehicle entered construction zone',
                            'penalty': 100  # 直接0分
                        })
                        total_penalty = 100  # 直接设为0分
                    break

            # 如果已进入施工区，停止进一步检查（已经0分）
            if entered_construction:
                continue

            # 检查施工区内超速（如果允许在施工区通行）
            if in_construction_zone:
                excess_speed = max(0.0, v - self.speed_limit)
                if excess_speed > 0:
                    # 计算扣分：向下取整超速量 * 每m/s每帧扣分
                    frame_penalty = math.floor(excess_speed) * self.penalty_per_frame_per_mps
                    total_penalty += frame_penalty

                    violations.append({
                        'type': 'CONSTRUCTION_SPEED_VIOLATION',
                        'frame': frame_idx,
                        'timestamp': timestamp,
                        'speed': v,
                        'excess_speed': excess_speed,
                        'speed_limit': self.speed_limit,
                        'frame_penalty': frame_penalty,
                        'message': f'Speed {v:.2f} m/s exceeds limit {self.speed_limit:.2f} m/s by {excess_speed:.2f} m/s'
                    })

        # 计算最终得分
        final_score = max(0, 100 - total_penalty)

        return {
            'scenario': 'construction_zone',
            'total_frames': len(trajectory_frames),
            'entered_construction_zone': entered_construction,
            'total_penalty': total_penalty,
            'final_score': final_score,
            'violations': violations,
            'violation_count': len(violations)
        }

    def _point_in_zone(self, x: float, y: float, zone: Dict[str, Any]) -> bool:
        """
        检查点是否在施工区内

        Args:
            x, y: 点的坐标
            zone: 施工区定义

        Returns:
            是否在施工区内
        """
        zone_type = zone.get('type', 'box')

        if zone_type == 'box':
            # 矩形区域
            return self._point_in_box(x, y, zone)
        elif zone_type == 'circle':
            # 圆形区域
            return self._point_in_circle(x, y, zone)
        elif zone_type == 'polygon':
            # 多边形区域
            return self._point_in_polygon(x, y, zone)
        else:
            # 默认当作矩形处理
            return self._point_in_box(x, y, zone)

    def _point_in_box(self, x: float, y: float, zone: Dict[str, Any]) -> bool:
        """检查点是否在矩形区域内"""
        xmin = zone.get('xmin', zone.get('x_min', -float('inf')))
        xmax = zone.get('xmax', zone.get('x_max', float('inf')))
        ymin = zone.get('ymin', zone.get('y_min', -float('inf')))
        ymax = zone.get('ymax', zone.get('y_max', float('inf')))

        return xmin <= x <= xmax and ymin <= y <= ymax

    def _point_in_circle(self, x: float, y: float, zone: Dict[str, Any]) -> bool:
        """检查点是否在圆形区域内"""
        center_x = zone.get('center_x', 0.0)
        center_y = zone.get('center_y', 0.0)
        radius = zone.get('radius', 0.0)

        dx = x - center_x
        dy = y - center_y
        distance = math.sqrt(dx*dx + dy*dy)

        return distance <= radius

    def _point_in_polygon(self, x: float, y: float, zone: Dict[str, Any]) -> bool:
        """检查点是否在多边形区域内（使用射线法）"""
        points = zone.get('points', [])
        if len(points) < 3:
            return False

        # 射线法判断点是否在多边形内
        n = len(points)
        inside = False

        p1x, p1y = points[0]
        for i in range(1, n + 1):
            p2x, p2y = points[i % n]
            if y > min(p1y, p2y):
                if y <= max(p1y, p2y):
                    if x <= max(p1x, p2x):
                        if p1y != p2y:
                            xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                        if p1x == p2x or x <= xinters:
                            inside = not inside
            p1x, p1y = p2x, p2y

        return inside

    def print_results(self, results: Dict[str, Any]) -> None:
        """打印评分结果"""
        print("\n" + "="*60)
        print("🏗️  施工区域通行评分结果")
        print("="*60)

        print(f"📊 得分: {results['final_score']}/100")
        print(f"⏱️  总帧数: {results['total_frames']}")

        if results['entered_construction_zone']:
            print("❌ 结果: 失败 (驶入施工区域)")
        else:
            print("✅ 结果: 通过" if results['final_score'] == 100 else "⚠️  结果: 通过但有扣分")

        if results['violations']:
            print(f"\n🚫 违规记录 ({len(results['violations'])}项):")
            for violation in results['violations'][:10]:  # 只显示前10项
                print(f"  • {violation['type']}: {violation['message']}")
                if violation['type'] == 'CONSTRUCTION_ZONE_ENTRY':
                    print(f"    📍 位置: ({violation['x']:.2f}, {violation['y']:.2f})")
                elif violation['type'] == 'CONSTRUCTION_SPEED_VIOLATION':
                    print(f"    🚗 速度: {violation['speed']:.2f} m/s (限速: {violation['speed_limit']:.2f} m/s)")
                    print(f"    💰 扣分: -{violation['frame_penalty']}")

            if len(results['violations']) > 10:
                print(f"  ... 还有 {len(results['violations']) - 10} 项违规")

        print(f"\n💰 总扣分: -{results['total_penalty']}")
        print(f"🏆 最终得分: {results['final_score']}")


def load_trajectory_from_json(json_file: str) -> List[Dict[str, Any]]:
    """从JSON文件加载轨迹数据"""
    try:
        with open(json_file, 'r') as f:
            data = json.load(f)

        if isinstance(data, list):
            return data
        elif isinstance(data, dict) and 'trajectory' in data:
            return data['trajectory']
        else:
            print(f"错误: JSON格式不正确")
            return []
    except Exception as e:
        print(f"错误: 读取轨迹文件失败 {e}")
        return []


def load_construction_zones_from_json(json_file: str) -> List[Dict[str, Any]]:
    """从JSON文件加载施工区定义"""
    try:
        with open(json_file, 'r') as f:
            data = json.load(f)

        if isinstance(data, list):
            return data
        elif isinstance(data, dict) and 'construction_zones' in data:
            return data['construction_zones']
        else:
            print(f"错误: 施工区定义格式不正确")
            return []
    except Exception as e:
        print(f"错误: 读取施工区文件失败 {e}")
        return []


def main():
    """主函数"""
    import argparse

    parser = argparse.ArgumentParser(description='Apollo施工区评分器')
    parser.add_argument('--trajectory', required=True,
                       help='轨迹数据JSON文件')
    parser.add_argument('--construction-zones', required=True,
                       help='施工区定义JSON文件')
    parser.add_argument('--speed-limit', type=float, default=8.333,
                       help='施工区限速 (m/s), 默认30km/h')
    parser.add_argument('--penalty-rate', type=float, default=2.0,
                       help='每超速1m/s每帧扣分率')
    parser.add_argument('--output', type=str, default=None,
                       help='结果输出文件 (JSON格式)')

    args = parser.parse_args()

    # 加载数据
    trajectory = load_trajectory_from_json(args.trajectory)
    if not trajectory:
        print("错误: 无法加载轨迹数据")
        sys.exit(1)

    construction_zones = load_construction_zones_from_json(args.construction_zones)
    if not construction_zones:
        print("警告: 未找到施工区定义，将按无施工区评分")

    # 创建评分器并评分
    scorer = ConstructionZoneScorer(args.speed_limit, args.penalty_rate)
    results = scorer.score_trajectory(trajectory, construction_zones)

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
