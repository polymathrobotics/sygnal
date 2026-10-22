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

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror=switch)
  add_link_options("-Wl,--no-undefined")
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Header-only target for sygnal_types.hpp, for packages that need the types and not the CAN interface.
add_library(sygnal_types INTERFACE)

# The headers use C++20, so every consumer compiles with at least C++20.
target_compile_features(sygnal_types INTERFACE cxx_std_20)

target_include_directories(sygnal_types INTERFACE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)

target_link_libraries(sygnal_types INTERFACE magic_enum::magic_enum)

add_library(
  ${PROJECT_NAME} SHARED
  src/crc8.cpp
  src/sygnal_device.cpp
  src/sygnal_hpo_device.cpp
  src/sygnal_interface_socketcan.cpp
  src/sygnal_mcm_device.cpp
)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
    dbc_gen_cpp::dbc_gen_cpp
    socketcan_adapter::socketcan_adapter
    sygnal_dbc::sygnal_dbc
    sygnal_types
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME} sygnal_types
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
  dbc_gen_cpp
  magic_enum
  socketcan_adapter
  sygnal_dbc
)

if(BUILD_TESTING)
  ament_auto_find_test_dependencies()
  find_package(Catch2 REQUIRED)

  foreach(test_name
    response_queues
    sygnal_hpo_device
    sygnal_interface_socketcan_routing
    sygnal_mcm_device
  )
    add_executable(${test_name}_tests test/${test_name}_test.cpp)
    target_link_libraries(${test_name}_tests PRIVATE ${PROJECT_NAME} Catch2::Catch2WithMain)
    ament_add_test(
      ${test_name}_tests
      GENERATE_RESULT_FOR_RETURN_CODE_ZERO
      COMMAND "$<TARGET_FILE:${test_name}_tests>"
        -r junit -s
        -o test_results/${PROJECT_NAME}/${test_name}_tests_output.xml
      ENV CATCH_CONFIG_CONSOLE_WIDTH=120
      WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
    )
  endforeach()

  if(DEFINED ENV{CAN_AVAILABLE})
    add_executable(sygnal_interface_socketcan_tests
      test/sygnal_interface_socketcan_test.cpp
    )
    target_link_libraries(sygnal_interface_socketcan_tests
      PRIVATE ${PROJECT_NAME} Catch2::Catch2WithMain
    )
    ament_add_test(
      sygnal_interface_socketcan_tests
      GENERATE_RESULT_FOR_RETURN_CODE_ZERO
      COMMAND "$<TARGET_FILE:sygnal_interface_socketcan_tests>"
        -r junit -s
        -o test_results/${PROJECT_NAME}/sygnal_interface_socketcan_tests_output.xml
      ENV CATCH_CONFIG_CONSOLE_WIDTH=120
      WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
    )
    message(STATUS "CAN_AVAILABLE set - including hardware tests")
  endif()
endif()

ament_package()
