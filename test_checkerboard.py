#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
单元测试 - 棋盘格标定程序
Unit Tests for Checkerboard Calibration
"""

import unittest
import numpy as np
import os
from checkerboard_calibration import CheckerboardCalibration


class TestCheckerboardCalibration(unittest.TestCase):
    """测试棋盘格标定类"""
    
    def setUp(self):
        """初始化测试"""
        self.calibration = CheckerboardCalibration(pattern_size=(11, 8), square_size=3.5)
    
    def test_initialization(self):
        """测试初始化参数"""
        self.assertEqual(self.calibration.pattern_size, (11, 8))
        self.assertEqual(self.calibration.square_size, 3.5)
    
    def test_generate_3d_points(self):
        """测试3D点生成"""
        points = self.calibration.generate_3d_points()
        
        # 验证点的数量
        expected_count = 11 * 8
        self.assertEqual(points.shape[0], expected_count)
        
        # 验证点的维度
        self.assertEqual(points.shape[1], 3)
        
        # 验证第一个点在原点
        self.assertTrue(np.allclose(points[0], [0, 0, 0]))
        
        # 验证点之间的距离
        distance = np.linalg.norm(points[1] - points[0])
        self.assertAlmostEqual(distance, 3.5, places=5)
    
    def test_rotation_matrix_identity(self):
        """测试零旋转生成单位矩阵"""
        R = self.calibration.rotation_matrix_from_angles(0, 0, 0)
        identity = np.eye(3)
        self.assertTrue(np.allclose(R, identity))
    
    def test_rotation_matrix_properties(self):
        """测试旋转矩阵的性质"""
        R = self.calibration.rotation_matrix_from_angles(45, 30, 15)
        
        # 验证是正交矩阵 (R^T * R = I)
        result = R.T @ R
        identity = np.eye(3)
        self.assertTrue(np.allclose(result, identity, atol=1e-10))
        
        # 验证行列式为1 (保持方向)
        det = np.linalg.det(R)
        self.assertAlmostEqual(det, 1.0, places=10)
    
    def test_transform_points_identity(self):
        """测试零变换保持点不变"""
        points = self.calibration.generate_3d_points()
        transformed = self.calibration.transform_points(points, 0, 0, 0, 0, 0, 0)
        self.assertTrue(np.allclose(points, transformed))
    
    def test_transform_points_translation(self):
        """测试平移变换"""
        points = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0]])
        transformed = self.calibration.transform_points(points, 5, 10, 15, 0, 0, 0)
        
        expected = points + np.array([5, 10, 15])
        self.assertTrue(np.allclose(transformed, expected))
    
    def test_visualize_checkerboard_creates_image(self):
        """测试图像生成"""
        output_path = 'test_output.png'
        
        # 清理可能存在的旧文件
        if os.path.exists(output_path):
            os.remove(output_path)
        
        # 生成图像
        img = self.calibration.visualize_checkerboard(
            0, 0, 100, 0, 0, 0, output_path
        )
        
        # 验证图像存在
        self.assertTrue(os.path.exists(output_path))
        
        # 验证图像尺寸
        self.assertEqual(img.shape, (960, 1280, 3))
        
        # 清理测试文件
        if os.path.exists(output_path):
            os.remove(output_path)
    
    def test_project_to_2d(self):
        """测试2D投影"""
        # 简单的3D点
        points_3d = np.array([
            [0, 0, 100],  # 在相机正前方
            [10, 0, 100],
            [0, 10, 100]
        ])
        
        points_2d = self.calibration.project_to_2d(points_3d)
        
        # 验证投影结果的维度
        self.assertEqual(points_2d.shape[0], 3)
        self.assertEqual(points_2d.shape[1], 2)
        
        # 第一个点应该接近图像中心 (640, 480)
        self.assertTrue(np.allclose(points_2d[0], [640, 480], atol=1))


def run_tests():
    """运行所有测试"""
    suite = unittest.TestLoader().loadTestsFromTestCase(TestCheckerboardCalibration)
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    return result.wasSuccessful()


if __name__ == '__main__':
    success = run_tests()
    exit(0 if success else 1)
