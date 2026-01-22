# C++ 手眼标定示例

说明：这是一个使用 OpenCV 提供的 `calibrateHandEye` 接口的最小示例。程序读取两个文本文件（机器人位姿与相机位姿），每行一个 4x4 变换（16 个数，行优先），然后计算手眼变换 X，使得 A_i * X = X * B_i。

构建（Windows，假设已安装 OpenCV 并配置到系统或使用 vcpkg）：

```bash
mkdir build && cd build
cmake -S .. -B . -DOpenCV_DIR=<path-to-opencv>/build
cmake --build . --config Release
```

运行：

```bash
./hand_eye_calibrate robot_poses.txt camera_poses.txt
```

输入文件格式示例（每行 16 个数，空格或逗号分隔）：

```
1 0 0 0  0 1 0 0  0 0 1 0  0 0 0 1
... (更多行)
```

注意：
- OpenCV 需包含 calib3d 模块，和版本 >= 3.4 推荐。
- 文件中每对变换的顺序应对应（第 i 行的机器人变换对应第 i 行的相机变换）。

示例数据与自动运行脚本：

- 本目录包含示例输入文件 [robot_poses.txt](cpp_hand_eye/robot_poses.txt#L1) 与 [camera_poses.txt](cpp_hand_eye/camera_poses.txt#L1)。
- 使用 Windows 时可运行 [run_example.bat](cpp_hand_eye/run_example.bat#L1) 来自动生成 `build`、构建并运行（需 CMake 可用且能找到 OpenCV）。
