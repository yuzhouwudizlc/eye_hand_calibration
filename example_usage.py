#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
示例用法: 展示如何使用棋盘格标定程序生成不同视角的棋盘格图像
Example Usage: Demonstrate how to generate checkerboard images from different viewpoints
"""

import os
from checkerboard_calibration import CheckerboardCalibration


def generate_multiple_views():
    """生成多个不同视角的棋盘格图像"""
    
    # 创建棋盘格标定对象 (11x8 内角点, 3.5cm方格)
    calibration = CheckerboardCalibration(pattern_size=(11, 8), square_size=3.5)
    
    # 定义不同的视角 (x, y, z, rx, ry, rz)
    viewpoints = [
        {
            'name': 'front_view',
            'x': 0, 'y': 0, 'z': 100,
            'rx': 0, 'ry': 0, 'rz': 0,
            'description': '正面视角'
        },
        {
            'name': 'tilted_view',
            'x': 10, 'y': 5, 'z': 100,
            'rx': 15, 'ry': 10, 'rz': 0,
            'description': '倾斜视角'
        },
        {
            'name': 'side_view',
            'x': 20, 'y': 0, 'z': 100,
            'rx': 0, 'ry': 30, 'rz': 0,
            'description': '侧面视角'
        },
        {
            'name': 'rotated_view',
            'x': 0, 'y': 10, 'z': 120,
            'rx': 10, 'ry': 20, 'rz': 15,
            'description': '旋转视角'
        },
        {
            'name': 'close_view',
            'x': -10, 'y': -5, 'z': 60,
            'rx': 5, 'ry': -10, 'rz': 5,
            'description': '近距离视角'
        }
    ]
    
    # 创建输出目录
    output_dir = 'output_views'
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    print("="*60)
    print("生成多视角棋盘格图像")
    print(f"棋盘格尺寸: {calibration.pattern_size[0]}x{calibration.pattern_size[1]} (内角点)")
    print(f"方格大小: {calibration.square_size} cm")
    print("="*60)
    
    # 生成每个视角的图像
    for i, view in enumerate(viewpoints, 1):
        print(f"\n[{i}/{len(viewpoints)}] 生成 {view['description']} ({view['name']})")
        print(f"   位置: x={view['x']}, y={view['y']}, z={view['z']}")
        print(f"   旋转: rx={view['rx']}°, ry={view['ry']}°, rz={view['rz']}°")
        
        output_path = os.path.join(output_dir, f"{view['name']}.png")
        calibration.visualize_checkerboard(
            view['x'], view['y'], view['z'],
            view['rx'], view['ry'], view['rz'],
            output_path
        )
    
    print("\n" + "="*60)
    print(f"完成! 所有图像已保存到 '{output_dir}/' 目录")
    print("="*60)


if __name__ == '__main__':
    generate_multiple_views()
