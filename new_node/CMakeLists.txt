cmake_minimum_required(VERSION 3.8)
project(behavior_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set C++20 standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(custom_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(nlohmann_json REQUIRED)

# Include directories
include_directories(include)
include_directories(/usr/include/behavior_lib)

# Source files
set(SOURCES
    src/main.cpp
    src/core/behavior_node_main.cpp
    src/core/behavior_executor.cpp
    src/core/message_queue.cpp
    src/data/data_cache.cpp
    src/data/mission_context.cpp
    src/data/ros_communication_manager.cpp
    src/data/ros_interface_definitions.cpp
    src/actions/flight_actions.cpp
    src/actions/navigation_actions.cpp
    src/actions/search_actions.cpp
    src/actions/control_actions.cpp
    src/actions/formation_actions.cpp
    src/conditions/flight_conditions.cpp
    src/conditions/navigation_conditions.cpp
    src/conditions/search_conditions.cpp
    src/conditions/system_conditions.cpp
    src/utils/task_validator.cpp
    src/utils/coordinate_converter.cpp
)

# Create executable
add_executable(${PROJECT_NAME} ${SOURCES})

# Dependencies
set(DEPENDENCIES
    rclcpp
    std_msgs
    geometry_msgs
    sensor_msgs
    mavros_msgs
    custom_msgs
    behaviortree_cpp
    nlohmann_json
)

ament_target_dependencies(${PROJECT_NAME} ${DEPENDENCIES})

# Link libraries
target_link_libraries(${PROJECT_NAME}
    core_plugin
)

# Install
install(TARGETS ${PROJECT_NAME}
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY tree
    DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()