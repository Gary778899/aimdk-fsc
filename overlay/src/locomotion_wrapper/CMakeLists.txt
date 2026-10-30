cmake_minimum_required(VERSION 3.8)
project(locomotion_wrapper)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(aimdk_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(hardware_wrapper_node_lib
  src/hardware_wrapper_node.cpp
)

target_include_directories(hardware_wrapper_node_lib
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(hardware_wrapper_node_lib
  aimdk_msgs
  rclcpp
  sensor_msgs
)

add_executable(hardware_wrapper_node
  src/hardware_wrapper_main.cpp
)

target_include_directories(hardware_wrapper_node
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(hardware_wrapper_node
  hardware_wrapper_node_lib
)

ament_target_dependencies(hardware_wrapper_node
  rclcpp
)

install(TARGETS hardware_wrapper_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  joint_watch.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME joint_watch
)

install(TARGETS hardware_wrapper_node_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(state_validation_test test/state_validation_test.cpp)
  target_include_directories(state_validation_test PRIVATE include)
  ament_target_dependencies(state_validation_test aimdk_msgs sensor_msgs)
endif()

ament_package()