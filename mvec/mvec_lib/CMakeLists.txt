# Copyright (c) 2025-present Polymath Robotics, Inc. All rights reserved
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
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

find_package(ament_cmake REQUIRED)
find_package(socketcan_adapter REQUIRED)

# socketcan adapter's improperly configured dependencies
# TODO (Zeerek): Remove when socketcan adapter's dependencies are cleaned up
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(can_msgs REQUIRED)

add_library(
  ${PROJECT_NAME} SHARED
  src/mvec_relay.cpp
  src/mvec_relay_socketcan.cpp
  src/status_messages/mvec_fuse_status_message.cpp
  src/status_messages/mvec_relay_status_message.cpp
  src/status_messages/mvec_error_status_message.cpp
  src/responses/mvec_response_base.cpp
  src/responses/mvec_relay_command_reply.cpp
  src/responses/mvec_relay_query_reply.cpp
  src/responses/mvec_population_reply.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)

target_link_libraries(${PROJECT_NAME}
  PUBLIC socketcan_adapter::socketcan_adapter
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}_TARGETS
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  EXPORT ${PROJECT_NAME}_TARGETS
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION share/${PROJECT_NAME}/cmake
)

ament_export_targets(${PROJECT_NAME}_TARGETS HAS_LIBRARY_TARGET)
ament_export_dependencies(
  socketcan_adapter
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  can_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(Catch2 2 REQUIRED)

  ament_lint_auto_find_test_dependencies()

  include(Catch)

  add_executable(relay_tests
    test/mvec_relay.cpp
  )
  target_link_libraries(relay_tests
    PRIVATE ${PROJECT_NAME} Catch2::Catch2WithMain
  )
  catch_discover_tests(relay_tests)

  add_executable(status_messages_tests
    test/status_messages.cpp
  )
  target_link_libraries(status_messages_tests
    PRIVATE ${PROJECT_NAME} Catch2::Catch2WithMain
  )
  catch_discover_tests(status_messages_tests)

  add_executable(status_enums_tests
    test/status_enums.cpp
  )
  target_link_libraries(status_enums_tests
    PRIVATE ${PROJECT_NAME} Catch2::Catch2WithMain
  )
  catch_discover_tests(status_enums_tests)

  # Only build hardware tests if CAN_AVAILABLE is set in the environment
  if(DEFINED ENV{CAN_AVAILABLE})
    add_executable(socketcan_hardware_tests
      test/mvec_socketcan_hardware.cpp
    )
    target_link_libraries(socketcan_hardware_tests
      PRIVATE ${PROJECT_NAME} Catch2::Catch2WithMain
    )
    catch_discover_tests(socketcan_hardware_tests)
    message(STATUS "CAN_AVAILABLE set - including hardware tests")
  else()
    message(STATUS "CAN_AVAILABLE not set - skipping hardware tests (set CAN_AVAILABLE=1 to enable)")
  endif()
endif()

ament_package()
