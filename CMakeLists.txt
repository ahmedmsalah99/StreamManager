cmake_minimum_required(VERSION 3.16)
project(stream_manager)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
if(DEFINED ENV{COLCON_PREFIX_PATH})
    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(sensor_msgs REQUIRED)
    find_package(cv_bridge REQUIRED)
    add_definitions(-DENABLE_GAZEBO)
    message("Building with colcon")
endif()

find_package(OpenCV REQUIRED)
find_package(iceoryx_posh REQUIRED)
find_package(iceoryx_hoofs REQUIRED)
find_package(yaml-cpp REQUIRED)

# Include directories
include_directories(include/stream_manager)
include_directories(${OpenCV_INCLUDE_DIRS})
if(DEFINED ENV{COLCON_PREFIX_PATH})
    include_directories(include/stream_manager_ros2)
endif()

# Library sources
set(LIBRARY_SOURCES
    src/config_manager.cpp
    src/frame_buffer.cpp
    src/video_source_base.cpp
    src/usb_video_source.cpp
    src/mavlink_video_source.cpp
    src/stream_manager.cpp
)
if(DEFINED ENV{COLCON_PREFIX_PATH})
    list(APPEND LIBRARY_SOURCES src/gazebo_video_source.cpp)
endif()

# Create the library
add_library(${PROJECT_NAME} SHARED ${LIBRARY_SOURCES})

# Link libraries
target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBS}
    iceoryx_posh::iceoryx_posh
    iceoryx_hoofs::iceoryx_hoofs
    yaml-cpp
)

if(DEFINED ENV{COLCON_PREFIX_PATH})
    # ROS2 dependencies
    ament_target_dependencies(${PROJECT_NAME}
        rclcpp
        sensor_msgs
        cv_bridge
    )
endif()

if(BUILD_EXAMPLES)
    add_executable(basic_usage examples/basic_usage.cpp)
    target_link_libraries(basic_usage ${PROJECT_NAME})
endif()


# Install targets with proper export
install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}Targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

install(DIRECTORY include/
    DESTINATION include/
)

install(DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config/
)

# Install the export targets
install(EXPORT ${PROJECT_NAME}Targets
    FILE ${PROJECT_NAME}Targets.cmake
    NAMESPACE ${PROJECT_NAME}::
    DESTINATION lib/cmake/${PROJECT_NAME}
)

# Generate and install package config files
include(CMakePackageConfigHelpers)

configure_package_config_file(
    cmake/${PROJECT_NAME}Config.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
    INSTALL_DESTINATION lib/cmake/${PROJECT_NAME}
)

write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
    VERSION 1.0.0
    COMPATIBILITY SameMajorVersion
)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
    DESTINATION lib/cmake/${PROJECT_NAME}
)

if(DEFINED ENV{COLCON_PREFIX_PATH})
    # Export dependencies
    ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
    ament_export_dependencies(
        rclcpp
        sensor_msgs
        cv_bridge
        OpenCV
        iceoryx_posh
        iceoryx_hoofs
        yaml-cpp
    )

    ament_package()
endif()