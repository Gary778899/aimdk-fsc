cmake_minimum_required(VERSION 3.8)
project(data_collection_edge_agent)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(mcap_vendor REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZeroMQ REQUIRED IMPORTED_TARGET libzmq)
find_path(CPPZMQ_INCLUDE_DIR zmq.hpp REQUIRED)
find_package(Threads REQUIRED)

add_executable(data_collection_edge_agent_node
  src/data_collection_edge_agent_node.cpp
)

target_compile_features(data_collection_edge_agent_node PRIVATE cxx_std_17)

target_include_directories(data_collection_edge_agent_node PRIVATE
  include
  ${CPPZMQ_INCLUDE_DIR}
)

target_link_libraries(data_collection_edge_agent_node
  PkgConfig::ZeroMQ Threads::Threads
)

ament_target_dependencies(data_collection_edge_agent_node
  rclcpp
  rmw
  sensor_msgs
  rosbag2_cpp
  rosbag2_storage
  mcap_vendor
)

install(TARGETS data_collection_edge_agent_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
