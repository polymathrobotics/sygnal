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

cmake_minimum_required(VERSION 3.8)
project(sygnal_can_interface_lib)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror=switch)
  add_link_options("-Wl,--no-undefined")
endif()

find_package(ament_cmake REQUIRED)
find_package(socketcan_adapter REQUIRED)
find_package(sygnal_dbc REQUIRED)

add_library(
  ${PROJECT_NAME} SHARED
  src/crc8.cpp
  src/sygnal_mcm_interface.cpp
  src/sygnal_command_interface.cpp
  src/sygnal_interface_socketcan.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)

target_link_libraries(${PROJECT_NAME}
  PUBLIC socketcan_adapter::socketcan_adapter
)

ament_target_dependencies(${PROJECT_NAME} PUBLIC sygnal_dbc)

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
  sygnal_dbc
)

if(BUILD_TESTING)
  find_package(Catch2 2 REQUIRED)
  include(Catch)

  add_executable(sygnal_mcm_interface_tests
    test/sygnal_mcm_interface_test.cpp
  )
  target_link_libraries(sygnal_mcm_interface_tests
    PRIVATE ${PROJECT_NAME} Catch2::Catch2WithMain
  )
  catch_discover_tests(sygnal_mcm_interface_tests)

  add_executable(sygnal_command_interface_tests
    test/sygnal_command_interface_test.cpp
  )
  target_link_libraries(sygnal_command_interface_tests
    PRIVATE ${PROJECT_NAME} Catch2::Catch2WithMain
  )
  catch_discover_tests(sygnal_command_interface_tests)

  if(DEFINED ENV{CAN_AVAILABLE})
    add_executable(sygnal_interface_socketcan_tests
      test/sygnal_interface_socketcan_test.cpp
    )
    target_link_libraries(sygnal_interface_socketcan_tests
      PRIVATE ${PROJECT_NAME} Catch2::Catch2WithMain
    )
    catch_discover_tests(sygnal_interface_socketcan_tests)
    message(STATUS "CAN_AVAILABLE set - including hardware tests")
  endif()
endif()

ament_package()
