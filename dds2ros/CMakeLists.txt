cmake_minimum_required(VERSION 3.12.4)
add_definitions(-DHAVE_LOG_NO_INFO=0 -DFASTDDS_ENFORCE_LOG_INFO=1 -D__INTERNALDEBUG=1 -D__DEBUG=1)
if(NOT CMAKE_VERSION VERSION_LESS 3.0)
    cmake_policy(SET CMP0048 NEW)
endif()

project(DDSCommu)

# Find requirements


find_package(Boost REQUIRED)

if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    message(STATUS "当前操作系统是 Linux")
  
else()
    message(STATUS "操作系统: ${CMAKE_SYSTEM_NAME}")
    find_package(fastcdr 2.3.0 EXACT REQUIRED)
    find_package(fastdds 3.2.0 EXACT REQUIRED)
    include_directories(inc inc/thirdpart msg ${fastcdr_INCLUDE_DIRS}  ${fastdds_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
endif()

                        



# Set include path

#include_directories($ENV{ROS_DIR}/include)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


message(STATUS "Configuring DDSCommu example...")
file(GLOB DDS_COMMU_SOURCES_CXX "msg/*.cxx")

add_library(dds_commu STATIC src/DDSCtrl.cpp src/DDSPubSub.cpp ${DDS_COMMU_SOURCES_CXX})
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
#set(fastcdr_DIR "/usr/local/fastdds3.2.0/lib/cmake/fastcdr")
find_package(fastcdr 2.3.0 EXACT REQUIRED)
find_package(fastdds 3.2.0 EXACT REQUIRED)
target_include_directories(dds_commu PRIVATE 
    /usr/local/fastdds3.2.0/include 
    ${fastdds_INCLUDE_DIRS}
    inc 
    inc/thirdpart 
    msg 
    )

target_link_directories(dds_commu PUBLIC
    /usr/local/fastdds3.2.0/lib
    )
target_link_libraries(dds_commu PUBLIC
fastdds
fastcdr 
)
else()
target_link_libraries(dds_commu fastdds fastcdr )
endif()


if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    add_definitions("-Wall -g")
    #find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(custom_msgs REQUIRED) 
    include_directories(inc inc/thirdpart msg   ${rclcpp_INCLUDE_DIRS} ${custom_msgs_INCLUDE_DIRS}  ${Boost_INCLUDE_DIRS})
    add_library(commu_api STATIC src/CommApi.cpp)
    target_link_libraries(commu_api dds_commu dislog)
    add_executable(DDS2ROS src/Ros2Ctrl.cpp src/DDS2RosNode.cpp)
    add_executable(TestSub src/TestNode.cpp)
    target_link_libraries(DDS2ROS PRIVATE dds_commu dislog ${rclcpp_LIBRARIES} ${custom_msgs_LIBRARIES})
    target_link_libraries(TestSub dislog ${rclcpp_LIBRARIES} ${custom_msgs_LIBRARIES})

else()
    add_library(commu_api SHARED src/CommApi.cpp)
    target_link_libraries(commu_api dds_commu )
endif()

add_executable(GrdControl src/GrdControl.cpp)
target_link_libraries(GrdControl commu_api json)




