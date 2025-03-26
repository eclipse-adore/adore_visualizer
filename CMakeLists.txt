cmake_minimum_required(VERSION 3.8)

get_filename_component(CURRENT_DIR_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
string(TOLOWER ${CURRENT_DIR_NAME} PROJECT)
project(${PROJECT})

set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE INTERNAL "")

include($ENV{SOURCE_DIRECTORY}/libraries/adore_libraries.cmake)

include(requirements.cmake)

# Ensure the Release build uses -O3 for optimization
if(CMAKE_BUILD_TYPE STREQUAL "Release")
  add_compile_options(-O3)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


add_executable(vehicle_visualizer_node 
  src/vehicle_visualizer.cpp
  src/visualizer_conversions.cpp
  src/visualization_primitives.cpp
  src/map_image_visualization.cpp
)

target_include_directories(vehicle_visualizer_node  PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_all_target_include_directories(vehicle_visualizer_node)
add_all_target_link_libraries(vehicle_visualizer_node)

add_executable(infrastructure_visualizer_node 
  src/infrastructure_visualizer.cpp
  src/visualizer_conversions.cpp
  src/visualization_primitives.cpp
  src/map_image_visualization.cpp
)

target_include_directories(infrastructure_visualizer_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_all_target_include_directories(infrastructure_visualizer_node )
add_all_target_link_libraries(infrastructure_visualizer_node )

install(TARGETS
    vehicle_visualizer_node   
    infrastructure_visualizer_node 
  DESTINATION lib/${PROJECT}
)

ament_package()
