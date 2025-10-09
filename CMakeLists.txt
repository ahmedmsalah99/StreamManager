cmake_minimum_required(VERSION 3.16)
project(stream_manager)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(shm_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcpputils REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(message_filters REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Open3D)
find_package(Boost QUIET)
find_package(PCL REQUIRED QUIET COMPONENTS common io)
find_package(pcl_msgs REQUIRED)

find_package(OpenCV 4 QUIET)
if(NOT OpenCV_FOUND)
  find_package(OpenCV 3 REQUIRED)
endif()

find_package(iceoryx_posh CONFIG REQUIRED)
find_package(fastrtps CONFIG REQUIRED)
find_package(yaml-cpp REQUIRED)

# Define library sources
set(LIBRARY_SOURCES
    lib/src/config_manager.cpp
    lib/src/video_source_base.cpp
    lib/src/usb_video_source.cpp
    lib/src/mavlink_video_source.cpp
    lib/src/gazebo_video_source.cpp
)


# Create shared library
add_executable(${PROJECT_NAME}_node src/stream_manager.cpp)
# ROS 2 dependencies - this handles all ROS2 linking automatically
ament_target_dependencies(${PROJECT_NAME}_node
    rclcpp
    sensor_msgs
    cv_bridge
    "shm_msgs"
    builtin_interfaces
)
target_include_directories(${PROJECT_NAME}_node BEFORE PRIVATE
    ${shm_msgs_DIR}/../../../include
)
# get_target_property(INC_DIRS ${PROJECT_NAME}_node INCLUDE_DIRECTORIES)
# message(INFO "Include dirs for myTarget (direct): ${INC_DIRS}")


target_include_directories(${PROJECT_NAME}_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(${PROJECT_NAME}_library SHARED ${LIBRARY_SOURCES})

target_include_directories(${PROJECT_NAME}_library PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/lib/include>
  $<INSTALL_INTERFACE:include>)
# Link libraries
target_link_libraries(${PROJECT_NAME}_library
    ${OpenCV_LIBRARIES}
    yaml-cpp
)

target_link_libraries(${PROJECT_NAME}_node
    ${PROJECT_NAME}_library
)


ament_target_dependencies(${PROJECT_NAME}_library
    rclcpp
    sensor_msgs
    cv_bridge
    shm_msgs
    builtin_interfaces
)




# Optional example executable
if(BUILD_EXAMPLES)
    add_executable(basic_usage examples/basic_usage.cpp)
    target_link_libraries(basic_usage ${PROJECT_NAME})
    
    ament_target_dependencies(basic_usage
        rclcpp
        shm_msgs
    )
endif()

# Install library
install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)


install(TARGETS ${PROJECT_NAME}_library
  DESTINATION lib/${PROJECT_NAME}
)

# Install include directory
install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY lib/include/
  DESTINATION include/
)

# Install config directory if it exists
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/config)
  install(DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config/
  )
endif()

# Export information to downstream packages
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
    shm_msgs
)
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
# Install targets
install(TARGETS ${PROJECT_NAME}_library
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()