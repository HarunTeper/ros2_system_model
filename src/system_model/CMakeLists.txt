cmake_minimum_required(VERSION 3.5)
project(system_model)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

# Include Cpp "include" directory
include_directories(
  include
  ${YAML_CPP_INCLUDEDIR}
)

#Create libraries
add_library(composition_nodes SHARED
   src/sensor_node.cpp
   src/filter_node.cpp
   src/subscription_actuator_node.cpp
   src/timer_actuator_node.cpp
   src/subscription_fusion_node.cpp
   src/timer_fusion_node.cpp
   )
target_compile_definitions(composition_nodes
  PRIVATE "COMPOSITION_NODES_DLL")
ament_target_dependencies(composition_nodes rclcpp rclcpp_components std_msgs yaml-cpp)
target_link_libraries(composition_nodes ${YAML_CPP_LIBRARIES})

# Create Cpp executable
add_executable(system_composition src/system_composition.cpp)
target_link_libraries(system_composition composition_nodes)
ament_target_dependencies(system_composition rclcpp)

# Install libraries
install(TARGETS
  composition_nodes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Install Cpp executables
install(TARGETS
  system_composition
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()