cmake_minimum_required(VERSION 3.5)
project(orb_slam3_ros)

# # You should set the PYTHONPATH to your own python site-packages path
# set(ENV{PYTHONPATH} "/opt/ros/humble/lib/python3.10/site-packages/")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_definitions(-w)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)


set(LIBS_ORBSLAM
    ${OpenCV_LIBS}
    ${EIGEN3_LIBS}
    ${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty/DBoW2/lib/libDBoW2.so
    ${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty/g2o/lib/libg2o.so
)



rosidl_generate_interfaces(${PROJECT_NAME}
  "ros2/srv/SaveMap.srv"
  DEPENDENCIES std_msgs
)


include_directories(
${PROJECT_SOURCE_DIR}/orb_slam3
${PROJECT_SOURCE_DIR}/orb_slam3/include
${PROJECT_SOURCE_DIR}/orb_slam3/include/CameraModels
${PROJECT_SOURCE_DIR}/ros2/include
${EIGEN3_INCLUDE_DIR}
${SOPHUS_INCLUDE_DIR}
)

include(${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty/DBoW2/CMakeLists.txt)
include(${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty/g2o/CMakeLists.txt)
# include(${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty/Sophus/CMakeLists.txt)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/orb_slam3/lib)


file(GLOB_RECURSE ORBSLAM_SRC_FILES CONFIGURE_DEPENDS
  ${CMAKE_CURRENT_SOURCE_DIR}/orb_slam3/src/*.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/orb_slam3/src/*.cpp
)

file(GLOB_RECURSE ORBSLAM_HEADER_FILES CONFIGURE_DEPENDS
  ${CMAKE_CURRENT_SOURCE_DIR}/orb_slam3/include/*.h
  ${CMAKE_CURRENT_SOURCE_DIR}/orb_slam3/include/CameraModels/*.h
)

add_library(${PROJECT_NAME}_core SHARED
${ORBSLAM_SRC_FILES}
)

# ament_target_dependencies(${PROJECT_NAME}_core ${LIBS_ORBSLAM} )

target_link_libraries(${PROJECT_NAME}_core ${LIBS_ORBSLAM}
)


add_executable(${PROJECT_NAME}_mono
  ros2/src/monocular_node.cpp
  ros2/src/node.cpp
)
ament_target_dependencies(${PROJECT_NAME}_mono rclcpp sensor_msgs cv_bridge tf2_ros std_msgs tf2_geometry_msgs)
target_link_libraries(${PROJECT_NAME}_mono
        ${PROJECT_NAME}_core
)


install(TARGETS ${PROJECT_NAME}_mono
  DESTINATION lib/${PROJECT_NAME})

# Install launch files.
#install(DIRECTORY launch config vocabulary
#  DESTINATION share/${PROJECT_NAME}/)

ament_package()

