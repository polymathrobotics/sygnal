cmake_minimum_required(VERSION 3.8)
project(mvec_ros2)

# Project-wide language standards
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Warnings & link hygiene
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  add_link_options(-Wl,-no-undefined)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(magic_enum REQUIRED)
find_package(mvec_msgs REQUIRED)
find_package(mvec_lib REQUIRED)
find_package(socketcan_adapter REQUIRED)

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  message(STATUS "Enabling AddressSanitizer (ASAN) for Debug build")
  set(ASAN_FLAGS "-fsanitize=address -fno-omit-frame-pointer")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${ASAN_FLAGS}")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${ASAN_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${ASAN_FLAGS}")
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} ${ASAN_FLAGS}")
endif()

# Primary component library (renamed to avoid conflict with executable)
add_library(mvec_ros2 SHARED
  src/mvec_node.cpp
)
target_include_directories(mvec_ros2 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_definitions(mvec_ros2 PRIVATE "MVEC_NODE_BUILDING_LIBRARY")

target_link_libraries(mvec_ros2 PUBLIC
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  rclcpp_components::component
  ${diagnostic_msgs_TARGETS}
  magic_enum::magic_enum
  ${mvec_msgs_TARGETS}
  mvec_lib::mvec_lib
  socketcan_adapter::socketcan_adapter
)

# Generate composable node executable (renamed from mvec_bridge to mvec_node)
rclcpp_components_register_node(mvec_ros2
  PLUGIN "polymath::mvec::MvecNode"
  EXECUTABLE mvec_node)

# Installs
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

install(TARGETS mvec_ros2
  EXPORT ${PROJECT_NAME}_TARGETS
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS mvec_node DESTINATION lib/${PROJECT_NAME})
install(EXPORT ${PROJECT_NAME}_TARGETS
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION share/${PROJECT_NAME}/cmake
)

# Export
ament_export_targets(${PROJECT_NAME}_TARGETS HAS_LIBRARY_TARGET)
ament_package()

