# 使用示例 / Usage Examples

## 基本命令 / Basic Commands

### 1. 查看帮助信息
```bash
python checkerboard_calibration.py --help
```

### 2. 生成默认视角 (正面)
```bash
python checkerboard_calibration.py
```
输出: `checkerboard_view.png`

### 3. 指定位置和旋转
```bash
python checkerboard_calibration.py --x 10 --y 5 --z 100 --rx 15 --ry 10 --rz 0 --output my_view.png
```

### 4. 批量生成多个视角
```bash
python example_usage.py
```
输出目录: `output_views/` (包含5个不同视角的图像)

## 参数说明 / Parameter Description

### 位置参数 (Position Parameters)
- `--x`: X坐标，单位厘米 (默认: 0.0)
- `--y`: Y坐标，单位厘米 (默认: 0.0)
- `--z`: Z坐标，单位厘米 (默认: 100.0)
  - 推荐范围: 60-200cm

### 旋转参数 (Rotation Parameters)
- `--rx`: 绕X轴旋转角度，单位度 (默认: 0.0)
  - 正值: 向上倾斜
  - 负值: 向下倾斜
- `--ry`: 绕Y轴旋转角度，单位度 (默认: 0.0)
  - 正值: 向右旋转
  - 负值: 向左旋转
- `--rz`: 绕Z轴旋转角度，单位度 (默认: 0.0)
  - 正值: 顺时针旋转
  - 负值: 逆时针旋转

### 输出参数 (Output Parameter)
- `--output`: 输出图像路径 (默认: checkerboard_view.png)

## 实际应用场景 / Use Cases

### 场景1: 相机标定
为相机标定准备不同视角的标定板图像:
```bash
python example_usage.py
```

### 场景2: 视觉系统测试
测试视觉算法对不同位姿的检测能力:
```bash
# 近距离测试
python checkerboard_calibration.py --z 60 --rx 5 --ry -10

# 远距离测试
python checkerboard_calibration.py --z 150 --rx 10 --ry 15

# 大角度测试
python checkerboard_calibration.py --z 100 --rx 30 --ry 45
```

### 场景3: 教学演示
展示透视投影和3D变换:
```bash
# 演示旋转效果
python checkerboard_calibration.py --rx 0 --ry 0 --rz 0 --output demo_0deg.png
python checkerboard_calibration.py --rx 15 --ry 0 --rz 0 --output demo_15deg.png
python checkerboard_calibration.py --rx 30 --ry 0 --rz 0 --output demo_30deg.png
```

## 技术细节 / Technical Details

### 棋盘格规格
- 内角点: 11×8 = 88个点
- 方格大小: 3.5 cm × 3.5 cm
- 总尺寸: 38.5 cm × 28 cm

### 坐标系定义
- X轴: 向右为正
- Y轴: 向下为正
- Z轴: 远离相机为正

### 相机参数 (默认)
- 焦距: 800 像素
- 图像中心: (640, 480)
- 图像分辨率: 1280×960

## 运行测试 / Running Tests

```bash
# 运行所有单元测试
python test_checkerboard.py

# 期望输出: 8个测试全部通过
# Ran 8 tests in 0.027s
# OK
```

## 依赖项安装 / Install Dependencies

```bash
pip install -r requirements.txt
```

需要的包:
- numpy >= 1.21.0
- opencv-python >= 4.5.0
