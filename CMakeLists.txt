cmake_minimum_required(VERSION 3.8)
project(visualizer VERSION 0.1.0 LANGUAGES CXX)

# C++20 strict mode
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Compile commands for IDE support
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Optimization flags
if(CMAKE_BUILD_TYPE STREQUAL "Release")
  add_compile_options(-O3)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(adore_math REQUIRED)
find_package(adore_map REQUIRED)
find_package(adore_ros2_msgs REQUIRED)
find_package(adore_math_conversions REQUIRED)
find_package(adore_map_conversions REQUIRED)
find_package(adore_dynamics_conversions REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# Executable
add_executable(${PROJECT_NAME}
  src/visualizer_conversions.cpp
  src/visualization_primitives.cpp
  src/visualizer.cpp
  src/map_image_visualization.cpp
)

target_include_directories(${PROJECT_NAME}
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${OpenCV_INCLUDE_DIRS}

)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  std_msgs
  geometry_msgs
  visualization_msgs
  adore_math
  adore_map
  adore_ros2_msgs
  adore_math_conversions
  adore_map_conversions
  adore_dynamics_conversions
  tf2
  tf2_ros
  OpenCV
  cv_bridge
)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
