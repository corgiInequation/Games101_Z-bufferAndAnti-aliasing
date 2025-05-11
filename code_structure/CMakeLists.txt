cmake_minimum_required(VERSION 3.10)
project(Rasterizer)

# 设置 C++ 标准（全局）
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找 OpenCV 包（现代写法）
find_package(OpenCV REQUIRED)

# 查找 Eigen3 包（现代写法，需 Eigen3 3.3+）
find_package(Eigen3 REQUIRED)

# 定义可执行文件
add_executable(Rasterizer
    main.cpp
    rasterizer.hpp
    rasterizer.cpp
    Triangle.hpp
    Triangle.cpp
)

# 为可执行文件单独配置包含目录和链接库
target_include_directories(Rasterizer PRIVATE
    ${EIGEN3_INCLUDE_DIRS}  # Eigen3 头文件路径
    ${OpenCV_INCLUDE_DIRS}  # OpenCV 头文件路径
)

target_link_libraries(Rasterizer PRIVATE
    ${OpenCV_LIBRARIES}     # OpenCV 库
)
