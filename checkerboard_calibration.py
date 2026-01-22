#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
棋盘格标定板生成与可视化程序
Checkerboard Calibration Board Generator and Visualizer

棋盘格尺寸: 11x8 (内角点数量)
方格大小: 3.5cm
输入参数: x, y, z, rx, ry, rz (位置和旋转角度)
"""

import numpy as np
import cv2
import argparse
from typing import Tuple, List


class CheckerboardCalibration:
    """棋盘格标定板类"""
    
    def __init__(self, pattern_size=(11, 8), square_size=3.5):
        """
        初始化棋盘格标定板
        
        Args:
            pattern_size: 棋盘格内角点数量 (列数, 行数)
            square_size: 每个方格的大小 (cm)
        """
        self.pattern_size = pattern_size
        self.square_size = square_size
        
    def generate_3d_points(self) -> np.ndarray:
        """
        生成棋盘格的3D世界坐标点
        
        Returns:
            3D点坐标数组，形状为 (n_points, 3)
        """
        # 创建棋盘格角点的3D坐标 (假设Z=0平面)
        objp = np.zeros((self.pattern_size[0] * self.pattern_size[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:self.pattern_size[0], 0:self.pattern_size[1]].T.reshape(-1, 2)
        objp *= self.square_size
        return objp
    
    def rotation_matrix_from_angles(self, rx: float, ry: float, rz: float) -> np.ndarray:
        """
        从欧拉角生成旋转矩阵
        
        Args:
            rx, ry, rz: 绕X、Y、Z轴的旋转角度 (度)
            
        Returns:
            3x3旋转矩阵
        """
        # 转换为弧度
        rx_rad = np.deg2rad(rx)
        ry_rad = np.deg2rad(ry)
        rz_rad = np.deg2rad(rz)
        
        # 绕X轴旋转
        Rx = np.array([
            [1, 0, 0],
            [0, np.cos(rx_rad), -np.sin(rx_rad)],
            [0, np.sin(rx_rad), np.cos(rx_rad)]
        ])
        
        # 绕Y轴旋转
        Ry = np.array([
            [np.cos(ry_rad), 0, np.sin(ry_rad)],
            [0, 1, 0],
            [-np.sin(ry_rad), 0, np.cos(ry_rad)]
        ])
        
        # 绕Z轴旋转
        Rz = np.array([
            [np.cos(rz_rad), -np.sin(rz_rad), 0],
            [np.sin(rz_rad), np.cos(rz_rad), 0],
            [0, 0, 1]
        ])
        
        # 组合旋转矩阵: R = Rz * Ry * Rx
        R = Rz @ Ry @ Rx
        return R
    
    def transform_points(self, points: np.ndarray, x: float, y: float, z: float, 
                        rx: float, ry: float, rz: float) -> np.ndarray:
        """
        对3D点应用变换 (旋转和平移)
        
        Args:
            points: 原始3D点
            x, y, z: 平移向量 (cm)
            rx, ry, rz: 旋转角度 (度)
            
        Returns:
            变换后的3D点
        """
        R = self.rotation_matrix_from_angles(rx, ry, rz)
        t = np.array([x, y, z]).reshape(3, 1)
        
        # 应用旋转和平移: p' = R*p + t
        transformed = (R @ points.T + t).T
        return transformed
    
    def project_to_2d(self, points_3d: np.ndarray, 
                     camera_matrix: np.ndarray = None) -> np.ndarray:
        """
        将3D点投影到2D图像平面
        
        Args:
            points_3d: 3D点坐标
            camera_matrix: 相机内参矩阵 (如果为None，使用默认值)
            
        Returns:
            2D投影点
        """
        if camera_matrix is None:
            # 默认相机内参
            focal_length = 800
            cx, cy = 640, 480
            camera_matrix = np.array([
                [focal_length, 0, cx],
                [0, focal_length, cy],
                [0, 0, 1]
            ])
        
        # 透视投影
        points_2d = []
        for point in points_3d:
            if point[2] != 0:  # 避免除以零
                x_proj = (camera_matrix[0, 0] * point[0] / point[2]) + camera_matrix[0, 2]
                y_proj = (camera_matrix[1, 1] * point[1] / point[2]) + camera_matrix[1, 2]
                points_2d.append([x_proj, y_proj])
            else:
                points_2d.append([0, 0])
        
        return np.array(points_2d)
    
    def visualize_checkerboard(self, x: float, y: float, z: float,
                              rx: float, ry: float, rz: float,
                              output_path: str = None) -> np.ndarray:
        """
        可视化给定位姿下的棋盘格
        
        Args:
            x, y, z: 位置 (cm)
            rx, ry, rz: 旋转角度 (度)
            output_path: 输出图像路径 (可选)
            
        Returns:
            可视化图像
        """
        # 生成3D点
        points_3d = self.generate_3d_points()
        
        # 应用变换
        transformed_points = self.transform_points(points_3d, x, y, z, rx, ry, rz)
        
        # 投影到2D
        points_2d = self.project_to_2d(transformed_points)
        
        # 创建图像
        img = np.ones((960, 1280, 3), dtype=np.uint8) * 255
        
        # 绘制棋盘格角点
        for i, point in enumerate(points_2d):
            x_coord = int(point[0])
            y_coord = int(point[1])
            if 0 <= x_coord < 1280 and 0 <= y_coord < 960:
                cv2.circle(img, (x_coord, y_coord), 5, (0, 0, 255), -1)
                # 可选: 添加点序号
                # cv2.putText(img, str(i), (x_coord+5, y_coord+5), 
                #            cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 0), 1)
        
        # 绘制连接线
        for row in range(self.pattern_size[1]):
            for col in range(self.pattern_size[0] - 1):
                idx1 = row * self.pattern_size[0] + col
                idx2 = row * self.pattern_size[0] + col + 1
                pt1 = tuple(points_2d[idx1].astype(int))
                pt2 = tuple(points_2d[idx2].astype(int))
                if (0 <= pt1[0] < 1280 and 0 <= pt1[1] < 960 and
                    0 <= pt2[0] < 1280 and 0 <= pt2[1] < 960):
                    cv2.line(img, pt1, pt2, (100, 100, 100), 1)
        
        for col in range(self.pattern_size[0]):
            for row in range(self.pattern_size[1] - 1):
                idx1 = row * self.pattern_size[0] + col
                idx2 = (row + 1) * self.pattern_size[0] + col
                pt1 = tuple(points_2d[idx1].astype(int))
                pt2 = tuple(points_2d[idx2].astype(int))
                if (0 <= pt1[0] < 1280 and 0 <= pt1[1] < 960 and
                    0 <= pt2[0] < 1280 and 0 <= pt2[1] < 960):
                    cv2.line(img, pt1, pt2, (100, 100, 100), 1)
        
        # 添加标注信息
        info_text = f"Pose: x={x:.1f}, y={y:.1f}, z={z:.1f}, rx={rx:.1f}, ry={ry:.1f}, rz={rz:.1f}"
        cv2.putText(img, info_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)
        
        # 保存图像
        if output_path:
            cv2.imwrite(output_path, img)
            print(f"图像已保存到: {output_path}")
        
        return img


def main():
    """主函数"""
    parser = argparse.ArgumentParser(
        description='棋盘格标定板生成器 - 11x8格子，每格3.5cm',
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    
    parser.add_argument('--x', type=float, default=0.0, 
                       help='X坐标 (cm), 默认=0.0')
    parser.add_argument('--y', type=float, default=0.0, 
                       help='Y坐标 (cm), 默认=0.0')
    parser.add_argument('--z', type=float, default=100.0, 
                       help='Z坐标 (cm), 默认=100.0')
    parser.add_argument('--rx', type=float, default=0.0, 
                       help='绕X轴旋转角度 (度), 默认=0.0')
    parser.add_argument('--ry', type=float, default=0.0, 
                       help='绕Y轴旋转角度 (度), 默认=0.0')
    parser.add_argument('--rz', type=float, default=0.0, 
                       help='绕Z轴旋转角度 (度), 默认=0.0')
    parser.add_argument('--output', type=str, default='checkerboard_view.png',
                       help='输出图像路径, 默认=checkerboard_view.png')
    
    args = parser.parse_args()
    
    # 创建棋盘格标定对象
    calibration = CheckerboardCalibration(pattern_size=(11, 8), square_size=3.5)
    
    # 生成可视化
    print(f"生成棋盘格视图...")
    print(f"位置: x={args.x}, y={args.y}, z={args.z}")
    print(f"旋转: rx={args.rx}°, ry={args.ry}°, rz={args.rz}°")
    
    img = calibration.visualize_checkerboard(
        args.x, args.y, args.z, args.rx, args.ry, args.rz, args.output
    )
    
    print("完成!")


if __name__ == '__main__':
    main()
