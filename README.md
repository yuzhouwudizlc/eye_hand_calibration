# 棋盘格标定板生成器 / Checkerboard Calibration Generator

## 项目简介 / Project Description

本项目用于生成棋盘格标定板的不同视角图像，用于相机标定和计算机视觉应用。

This project generates checkerboard calibration board images from different viewpoints for camera calibration and computer vision applications.

### 棋盘格规格 / Checkerboard Specifications
- **尺寸 / Size**: 11×8 (内角点数量 / internal corner points)
- **方格大小 / Square Size**: 3.5 cm
- **输入参数 / Input Parameters**: x, y, z, rx, ry, rz
  - x, y, z: 位置坐标 (cm) / Position coordinates (cm)
  - rx, ry, rz: 旋转角度 (度) / Rotation angles (degrees)

## 安装 / Installation

### 依赖项 / Dependencies

```bash
pip install -r requirements.txt
```

需要的包 / Required packages:
- numpy >= 1.21.0
- opencv-python >= 4.5.0

## 使用方法 / Usage

### 基本用法 / Basic Usage

```bash
# 生成默认视角的棋盘格图像
python checkerboard_calibration.py

# 指定位置和旋转参数
python checkerboard_calibration.py --x 10 --y 5 --z 100 --rx 15 --ry 10 --rz 0 --output my_view.png
```

### 参数说明 / Parameters

- `--x`: X坐标 (cm), 默认=0.0
- `--y`: Y坐标 (cm), 默认=0.0
- `--z`: Z坐标 (cm), 默认=100.0
- `--rx`: 绕X轴旋转角度 (度), 默认=0.0
- `--ry`: 绕Y轴旋转角度 (度), 默认=0.0
- `--rz`: 绕Z轴旋转角度 (度), 默认=0.0
- `--output`: 输出图像路径, 默认=checkerboard_view.png

### 批量生成多视角图像 / Generate Multiple Views

运行示例脚本生成多个预定义视角的棋盘格图像：

```bash
python example_usage.py
```

这将在 `output_views/` 目录下生成多个不同视角的图像。

## 示例 / Examples

### 示例1: 正面视角
```bash
python checkerboard_calibration.py --x 0 --y 0 --z 100 --rx 0 --ry 0 --rz 0
```

### 示例2: 倾斜视角
```bash
python checkerboard_calibration.py --x 10 --y 5 --z 100 --rx 15 --ry 10 --rz 0
```

### 示例3: 近距离视角
```bash
python checkerboard_calibration.py --x -10 --y -5 --z 60 --rx 5 --ry -10 --rz 5
```

## 程序架构 / Program Structure

### CheckerboardCalibration 类

主要方法 / Main Methods:
- `generate_3d_points()`: 生成棋盘格的3D世界坐标点
- `rotation_matrix_from_angles(rx, ry, rz)`: 从欧拉角生成旋转矩阵
- `transform_points(points, x, y, z, rx, ry, rz)`: 对3D点应用变换
- `project_to_2d(points_3d)`: 将3D点投影到2D图像平面
- `visualize_checkerboard(x, y, z, rx, ry, rz, output_path)`: 可视化给定位姿下的棋盘格

## 文件说明 / File Description

- `checkerboard_calibration.py`: 主程序，包含棋盘格生成和可视化功能
- `example_usage.py`: 示例脚本，展示如何生成多视角图像
- `requirements.txt`: Python依赖包列表
- `README.md`: 项目说明文档

## 技术细节 / Technical Details

- 使用欧拉角表示旋转 (Rx, Ry, Rz)
- 旋转顺序: Rz * Ry * Rx
- 使用透视投影将3D点投影到2D平面
- 默认相机焦距: 800像素
- 默认图像尺寸: 1280×960

## License

MIT License
