cmake_minimum_required(VERSION 3.5)
project(behavior-process)

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
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(custom_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
file(GLOB SET_SRC "src/set/*.cpp")
file(GLOB GET_SRC "src/get/*.cpp")
file(GLOB CTR_SRC "src/control/*.cpp")
file(GLOB CHECK_SRC "src/check/*.cpp")
file(GLOB MGR_SRC "src/manage/*.cpp")
file(GLOB STS_SRC "src/status/*.cpp")
file(GLOB ACT_SRC "src/action_libs/*.cpp")
include_directories(${rclcpp_INCLUDE_DIRS} ${rclcpp_INCLUDE_DIRS}/include ${custom_msgs_INCLUDE_DIRS}
    ${custom_msgs_INCLUDE_DIRS}/include ${PROJECT_SOURCE_DIR}/include ${PROJECT_SOURCE_DIR}/include/action_libs
    ${std_msgs_INCLUDE_DIRS} ${std_msgs_INCLUDE_DIRS}/include /usr/include/behavior_lib
    ${sensor_msgs_INCLUDE_DIRS} ${sensor_msgs_INCLUDE_DIRS}/include ${mavros_msgs_INCLUDE_DIRS} ${mavros_msgs_INCLUDE_DIRS}/include
    ${PROJECT_SOURCE_DIR}/../fcu_comm/src/lib ${PROJECT_SOURCE_DIR}/../fcu_comm/src/lib/matrix)

add_library(trace_attack_plugin SHARED ${SET_SRC} ${GET_SRC} ${CTR_SRC} ${CHECK_SRC} ${MGR_SRC} ${STS_SRC}
    src/action_libs/trace_attack_plugin.cpp)

add_library(rally_plugin SHARED
    src/action_libs/rally_plugin.cpp
    src/set/SetRallyDst.cpp
)

add_executable(behavior-node src/behavior-node.cpp
    ../fcu_comm/src/lib/geo/geo.cpp)
ament_target_dependencies(behavior-node rclcpp std_msgs sensor_msgs mavros_msgs custom_msgs)

target_link_libraries(trace_attack_plugin core_plugin)
target_link_libraries(rally_plugin core_plugin)

target_link_libraries(behavior-node behaviortree_cpp ${rclcpp_LIBRARIES} json boost_filesystem core_plugin)
target_include_directories(behavior-node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

set_target_properties(trace_attack_plugin PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/tree
)

set_target_properties(rally_plugin PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/tree
)

# 将tree目录复制到${CMAKE_CURRENT_BINARY_DIR}/tree
add_custom_command(
    TARGET behavior-node POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/tree ${CMAKE_CURRENT_BINARY_DIR}/tree)


install(TARGETS trace_attack_plugin
    DESTINATION lib/${PROJECT_NAME}/tree)
install(TARGETS rally_plugin
    DESTINATION lib/${PROJECT_NAME}/tree)

install(TARGETS behavior-node
    DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY tree/ DESTINATION lib/${PROJECT_NAME}/tree FILES_MATCHING PATTERN "*.xml")

add_definitions("-Wall -g")
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
