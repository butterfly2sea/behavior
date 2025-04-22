cmake_minimum_required(VERSION 3.5)
project(behavior_lib)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

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

file (GLOB SET_SRC "src/set/*.cpp")
file (GLOB GET_SRC "src/get/*.cpp")
file (GLOB CTR_SRC "src/control/*.cpp")
file (GLOB CHECK_SRC "src/check/*.cpp")
file (GLOB UTILITY_SRC "src/utility/*.cpp")
file (GLOB MGR_SRC "src/manage/*.cpp")
file (GLOB STS_SRC "src/status/*.cpp")
file (GLOB PLUG_SRC "src/plugin/*.cpp")

include_directories(${rclcpp_INCLUDE_DIRS} ${rclcpp_INCLUDE_DIRS}/include ${custom_msgs_INCLUDE_DIRS} 
${custom_msgs_INCLUDE_DIRS}/include ${PROJECT_SOURCE_DIR}/include ${PROJECT_SOURCE_DIR}/include/plugin
${std_msgs_INCLUDE_DIRS} ${std_msgs_INCLUDE_DIRS}/include
${sensor_msgs_INCLUDE_DIRS} ${sensor_msgs_INCLUDE_DIRS}/include ${mavros_msgs_INCLUDE_DIRS} ${mavros_msgs_INCLUDE_DIRS}/include
${PROJECT_SOURCE_DIR}/../fcu_comm/src/lib ${PROJECT_SOURCE_DIR}/../fcu_comm/src/lib/matrix)

add_library(core_plugin SHARED ${SET_SRC} ${GET_SRC} ${CTR_SRC}
${UTILITY_SRC} ${CHECK_SRC} ${MGR_SRC} ${STS_SRC} ${PLUG_SRC} src/param.cpp )
ament_target_dependencies(core_plugin rclcpp std_msgs sensor_msgs mavros_msgs custom_msgs)
target_link_libraries(core_plugin json boost_filesystem dislog)

add_custom_command(
TARGET core_plugin POST_BUILD
COMMAND echo $ENV{USER} | sudo -S cp $<TARGET_FILE:core_plugin> /usr/lib/)

add_custom_command(
  TARGET core_plugin POST_BUILD
    COMMAND echo $ENV{USER} | sudo -S mkdir -p /usr/include/behavior_lib
    COMMAND echo $ENV{USER} | sudo -S cp -fr ${CMAKE_CURRENT_SOURCE_DIR}/include/* /usr/include/behavior_lib)
#install(DIRECTORY include/ DESTINATION /usr/include/behavior_lib
#    FILES_MATCHING PATTERN "*.*")
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
