cmake_minimum_required(VERSION 3.16)
project(behavior_node)

# Default to C99
if (NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif ()

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
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

file(GLOB_RECURSE ACTION_SRC "src/action/*.cpp")
file(GLOB_RECURSE COMMON_SRC "src/common/*.cpp")
file(GLOB_RECURSE COMM_SRC "src/communication/*.cpp")
file(GLOB_RECURSE CONDITION_SRC "src/condition/*.cpp")
file(GLOB_RECURSE INFO_SRC "src/information/*.cpp")

include_directories(
        include
        ${rclcpp_INCLUDE_DIRS}
        ${behaviortree_cpp_INCLUDE_DIRS}
)

add_executable(behavior_node
        src/behavior_node.cpp
        ${ACTION_SRC}
        ${CONDITION_SRC}
        ${COMMON_SRC}
        ${COMM_SRC}
        ${INFO_SRC}
)

target_include_directories(behavior_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

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

install(TARGETS behavior_node
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY trees/
        DESTINATION lib/${PROJECT_NAME}/trees
        FILES_MATCHING PATTERN "*.xml"
)

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # uncomment the line when a copyright and license is not present in all source files
    #set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # uncomment the line when this package is not in a git repo
    #set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()

ament_package()
