cmake_minimum_required(VERSION 3.8)
project(data_collection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(aimdk_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(mcap_vendor REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZeroMQ REQUIRED IMPORTED_TARGET libzmq)
find_path(CPPZMQ_INCLUDE_DIR zmq.hpp)
if(NOT CPPZMQ_INCLUDE_DIR)
  message(FATAL_ERROR
    "Missing zmq.hpp. On Ubuntu 22.04, install libzmq3-dev, which provides the C++ header. "
    "On distributions that package the C++ headers separately, install cppzmq-dev as well.")
endif()
find_package(Threads REQUIRED)

# 仅记录构建时可核实的版本；缺少源码仓库或版本文件时明确使用 unknown。
set(COLLECTION_SDK_VERSION "unknown")
set(AIMDK_SDK_VERSION_FILE "/home/agi/aimdk/version" CACHE FILEPATH
  "Version file of the fixed SDK underlay used for this build")
if(EXISTS "${AIMDK_SDK_VERSION_FILE}")
  file(READ "${AIMDK_SDK_VERSION_FILE}" COLLECTION_SDK_VERSION)
  string(STRIP "${COLLECTION_SDK_VERSION}" COLLECTION_SDK_VERSION)
endif()
set(COLLECTION_CODE_VERSION "unknown")
find_package(Git QUIET)
if(GIT_FOUND)
  execute_process(
    COMMAND "${GIT_EXECUTABLE}" rev-parse HEAD
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    RESULT_VARIABLE COLLECTION_GIT_RESULT OUTPUT_VARIABLE COLLECTION_GIT_HEAD
    ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE
  )
  if(COLLECTION_GIT_RESULT EQUAL 0)
    execute_process(
      COMMAND "${GIT_EXECUTABLE}" status --porcelain --untracked-files=normal
      WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
      RESULT_VARIABLE COLLECTION_STATUS_RESULT OUTPUT_VARIABLE COLLECTION_GIT_STATUS
      ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    if(COLLECTION_STATUS_RESULT EQUAL 0)
      if(COLLECTION_GIT_STATUS STREQUAL "")
        set(COLLECTION_CODE_VERSION "${COLLECTION_GIT_HEAD}+clean")
      else()
        set(COLLECTION_CODE_VERSION "${COLLECTION_GIT_HEAD}+dirty")
      endif()
    else()
      set(COLLECTION_CODE_VERSION "${COLLECTION_GIT_HEAD}+state_unknown")
    endif()
  endif()
endif()
# 避免版本文本中的引号或反斜线破坏编译宏。
string(REPLACE "\\" "\\\\" COLLECTION_SDK_VERSION "${COLLECTION_SDK_VERSION}")
string(REPLACE "\"" "\\\"" COLLECTION_SDK_VERSION "${COLLECTION_SDK_VERSION}")

add_executable(data_collection_node
  src/data_collection_node.cpp
  src/state_mapper.cpp
  src/session_writer.cpp
)
target_compile_features(data_collection_node PRIVATE cxx_std_17)
target_compile_definitions(data_collection_node PRIVATE
  DATA_COLLECTION_SDK_VERSION="${COLLECTION_SDK_VERSION}"
  DATA_COLLECTION_CODE_VERSION="${COLLECTION_CODE_VERSION}"
)
target_include_directories(data_collection_node PRIVATE
  include
  ${CPPZMQ_INCLUDE_DIR}
)
target_link_libraries(data_collection_node
  PkgConfig::ZeroMQ Threads::Threads nlohmann_json::nlohmann_json
)
ament_target_dependencies(data_collection_node
  rclcpp rmw builtin_interfaces aimdk_msgs rosbag2_cpp rosbag2_storage mcap_vendor
)

install(TARGETS data_collection_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/collection_control.py
  DESTINATION lib/${PROJECT_NAME} RENAME collection_control)
install(PROGRAMS scripts/inspect_collection.py
  DESTINATION lib/${PROJECT_NAME} RENAME inspect_collection)

ament_package()
