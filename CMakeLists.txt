cmake_minimum_required(VERSION 3.10)
project(hand_eye_calibrate_cpp)

# 建议将 OpenCV_DIR 设置为包含 OpenCVConfig.cmake 的目录
# 通常是 build 文件夹，而不是 lib 文件夹
set(OpenCV_DIR "D:/opencv/opencv/build")


find_package(OpenCV REQUIRED)

# 验证是否找到正确的 OpenCV
if(OpenCV_FOUND)
    message(STATUS "找到 OpenCV 版本: ${OpenCV_VERSION}")
    message(STATUS "OpenCV 包含目录: ${OpenCV_INCLUDE_DIRS}")
    message(STATUS "OpenCV 库目录: ${OpenCV_LIB_DIRS}")
else()
    message(FATAL_ERROR "未找到 OpenCV，请检查 OpenCV_DIR 设置")
endif()

# 生成可执行文件
add_executable(hand_eye_calibrate main.cpp)

# 包含头文件目录
target_include_directories(hand_eye_calibrate PRIVATE ${OpenCV_INCLUDE_DIRS})

# 链接 OpenCV 库
target_link_libraries(hand_eye_calibrate ${OpenCV_LIBS})

# 设置 C++11 标准
set_target_properties(hand_eye_calibrate PROPERTIES 
    CXX_STANDARD 11 
    CXX_STANDARD_REQUIRED ON
)