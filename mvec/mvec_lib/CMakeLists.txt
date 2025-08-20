# Copyright (c) 2025-present Polymath Robotics, Inc. All rights reserved
# Proprietary. Any unauthorized copying, distribution, or modification of this software is strictly prohibited.
cmake_minimum_required(VERSION 3.8)
project(mvec_lib)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror=switch)
  add_link_options("-Wl,--no-undefined")
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(socketcan_adapter REQUIRED)

# Create library
add_library(
  ${PROJECT_NAME} SHARED
  src/mvec_relay.cpp
  src/mvec_status_messages.cpp
  src/mvec_fuse_status_message.cpp
  src/mvec_relay_status_message.cpp
  src/mvec_error_status_message.cpp
  src/mvec_response_parser.cpp
  src/mvec_relay_command_reply.cpp
  src/mvec_relay_query_reply.cpp
  src/mvec_population_reply.cpp
)

# Set include directories
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)

# Link dependencies
target_link_libraries(${PROJECT_NAME}
  PUBLIC socketcan_adapter::socketcan_adapter
)

# Install headers
install(
  DIRECTORY include/
  DESTINATION include
)

# Install library
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}_TARGETS
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install CMake export
install(
  EXPORT ${PROJECT_NAME}_TARGETS
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION share/${PROJECT_NAME}/cmake
)

# Export dependencies
ament_export_targets(${PROJECT_NAME}_TARGETS HAS_LIBRARY_TARGET)
ament_export_dependencies(socketcan_adapter)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(Catch2 2 REQUIRED)

  ament_lint_auto_find_test_dependencies()

  add_executable(${PROJECT_NAME}_test
    test/test_mvec_relay.cpp
    test/test_status_messages.cpp
    test/test_status_enums.cpp
  )
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME} Catch2::Catch2)
  target_include_directories(${PROJECT_NAME}_test PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  )

  # Add main function for Catch2 v2
  target_compile_definitions(${PROJECT_NAME}_test PRIVATE CATCH_CONFIG_MAIN)

  # Register the test with CTest
  include(CTest)
  include(Catch)
  catch_discover_tests(${PROJECT_NAME}_test)
endif()

ament_package()
