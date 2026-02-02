cmake_minimum_required(VERSION 3.10)
project(RRTPathPlanning)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 编译选项
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -Wall -Wextra")

# 源文件
file(GLOB SOURCES "src/*.cpp")
file(GLOB HEADERS "include/*.h")

# 创建可执行文件
add_executable(rrt_planning ${SOURCES} ${HEADERS})

# 包含目录
target_include_directories(rrt_planning PRIVATE include)

# 测试目标
enable_testing()
add_executable(rrt_test tests/test_rrt.cpp src/rrt.cpp)
target_include_directories(rrt_test PRIVATE include)
add_test(NAME RRTTest COMMAND rrt_test)