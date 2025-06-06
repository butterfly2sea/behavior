cmake_minimum_required(VERSION 3.16)
project(behavior_test_simulation)

# Default to C99
if (NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif ()

# Default to C++17
if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(custom_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)

include_directories(include)

# 创建仿真节点可执行文件
add_executable(simulation_node
        src/simulation_node.cpp
        src/vehicle_simulator.cpp
)

target_include_directories(simulation_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(simulation_node
        rclcpp
        std_msgs
        sensor_msgs
        geometry_msgs
        custom_msgs
        mavros_msgs
)

# 创建测试客户端
add_executable(test_client
        src/test_client.cpp
)

target_include_directories(test_client PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(test_client
        rclcpp
        std_msgs
        geometry_msgs
        custom_msgs
)

# 安装目标
install(TARGETS simulation_node test_client
        DESTINATION lib/${PROJECT_NAME})

# 安装测试行为树文件
install(DIRECTORY test_trees/
        DESTINATION lib/${PROJECT_NAME}/test_trees
        FILES_MATCHING PATTERN "*.xml"
)

# 安装启动文件
install(DIRECTORY launch/
        DESTINATION share/${PROJECT_NAME}/launch
        FILES_MATCHING PATTERN "*.py"
)

# 安装测试脚本
install(DIRECTORY scripts/
        DESTINATION lib/${PROJECT_NAME}/scripts
        FILES_MATCHING PATTERN "*.py"
        PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE
        GROUP_READ GROUP_EXECUTE
        WORLD_READ WORLD_EXECUTE
)

ament_package()