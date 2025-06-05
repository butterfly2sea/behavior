cmake_minimum_required(VERSION 3.16)
project(behavior_node)

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
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(custom_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# 收集源文件
file(GLOB_RECURSE ACTION_SRC "src/action/*.cpp")
file(GLOB_RECURSE CONDITION_SRC "src/condition/*.cpp")

include_directories(include)

# 创建可执行文件
add_executable(behavior_node
        src/behavior_node.cpp
        ${ACTION_SRC}
        ${CONDITION_SRC}
)

target_include_directories(behavior_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(behavior_node
        rclcpp
        rclcpp_action
        behaviortree_cpp
        std_msgs
        sensor_msgs
        custom_msgs
        mavros_msgs
        geometry_msgs
)

target_link_libraries(behavior_node dislog)

# 安装目标
install(TARGETS behavior_node
        DESTINATION lib/${PROJECT_NAME})

# 安装行为树文件
install(DIRECTORY trees/
        DESTINATION lib/${PROJECT_NAME}/trees
        FILES_MATCHING PATTERN "*.xml"
)

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif ()

ament_package()