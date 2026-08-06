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
cmake_minimum_required(VERSION 3.5)

project(sygnal_dbc VERSION 0.1.0 LANGUAGES CXX)

# Umbrella package: depends on the sibling per-subsystem packages that own the generated
# libraries, and keeps the aggregate view of the raw DBCs (database/ + SYGNAL_DBC_DIR).
find_package(ament_cmake REQUIRED)

# Re-assemble the database/<subsystem>/<Name>.dbc layout SYGNAL_DBC_DIR points at.
set(_sygnal_dbc_subsystems cb hpo io mcm vehicles)
foreach(_subsystem IN LISTS _sygnal_dbc_subsystems)
  install(
    DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../${_subsystem}/
    DESTINATION share/${PROJECT_NAME}/database/${_subsystem}
    FILES_MATCHING PATTERN "*.dbc"
  )
endforeach()

# Its own package (cb_niro_dcan/) since colcon will not descend into cb/, but the
# installed layout keeps it where consumers expect it.
install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../cb_niro_dcan/
  DESTINATION share/${PROJECT_NAME}/database/cb/niro_dcan
  FILES_MATCHING PATTERN "*.dbc"
)

# ---- Backwards compatibility -------------------------------------------------
# This package used to build one aggregate library and install every header as
# sygnal_dbc/<prefix>_<name>.h. Both are preserved below: an INTERFACE library named
# sygnal_dbc linking every generated library, and forwarding headers. C symbols were
# unchanged by the split (cantools was already given --database-name <prefix>_<name>),
# so existing consumers need no changes. New code should use <lib>/<lib>.hpp and link
# dbc_sygnal_<subsystem>::<lib> directly.
include(${CMAKE_CURRENT_SOURCE_DIR}/../cmake/dbc_package.cmake)

set(_compat_include_dir ${CMAKE_CURRENT_BINARY_DIR}/compat_include)
set(_compat_header_dir ${_compat_include_dir}/${PROJECT_NAME})
file(MAKE_DIRECTORY ${_compat_header_dir})

add_library(${PROJECT_NAME} INTERFACE)

# Sibling directory name doubles as the library prefix and the package name suffix.
set(_sygnal_dbc_directories cb cb_niro_dcan hpo io mcm vehicles)

foreach(_directory IN LISTS _sygnal_dbc_directories)
  set(_package "dbc_sygnal_${_directory}")
  set(_prefix "${_directory}")

  find_package(${_package} REQUIRED)

  file(GLOB _dbc_files CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/../${_directory}/*.dbc")
  foreach(_dbc ${_dbc_files})
    dbc_library_name(_lib_name "${_prefix}" "${_dbc}")
    foreach(_extension h hpp)
      file(WRITE "${_compat_header_dir}/${_lib_name}.${_extension}"
        "// Generated compatibility shim -- do not edit.\n"
        "// Prefer including \"${_lib_name}/${_lib_name}.${_extension}\" directly.\n"
        "#pragma once\n"
        "#include \"${_lib_name}/${_lib_name}.${_extension}\"\n")
    endforeach()
    target_link_libraries(${PROJECT_NAME} INTERFACE ${_package}::${_lib_name})
  endforeach()
endforeach()

target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${_compat_include_dir}>
  $<INSTALL_INTERFACE:include>
)

install(DIRECTORY ${_compat_header_dir} DESTINATION include)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  dbc_sygnal_cb
  dbc_sygnal_cb_niro_dcan
  dbc_sygnal_hpo
  dbc_sygnal_io
  dbc_sygnal_mcm
  dbc_sygnal_vehicles
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # Decodes the installed DBC directly, so it also covers the database/ install above.
  ament_add_pytest_test(test_mcm_heartbeat_python
    test/test_mcm_heartbeat.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )

  find_package(Catch2 REQUIRED)
  find_package(dbc_sygnal_mcm REQUIRED)

  add_executable(test_mcm_heartbeat_cpp test/test_mcm_heartbeat.cpp)
  target_link_libraries(test_mcm_heartbeat_cpp
    PRIVATE Catch2::Catch2WithMain dbc_sygnal_mcm::mcm_heartbeat
  )
  ament_add_test(
    test_mcm_heartbeat_cpp
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND "$<TARGET_FILE:test_mcm_heartbeat_cpp>"
      -r junit -s
      -o test_results/${PROJECT_NAME}/test_mcm_heartbeat_cpp_output.xml
    ENV CATCH_CONFIG_CONSOLE_WIDTH=120
    WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  )
endif()

ament_package(CONFIG_EXTRAS "sygnal_dbc-extras.cmake")
