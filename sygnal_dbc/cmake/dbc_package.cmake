# Copyright (c) 2025-present Polymath Robotics, Inc. All rights reserved
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# Shared helper for the per-subsystem DBC packages in this directory.
#
# dbc_package() turns every *.dbc in the calling package's source directory into an
# exported C/C++ CAN message library, installs the raw DBCs, then finishes the ament
# package, so adding a new .dbc needs NO cmake changes. Each CMakeLists.txt is just:
#
#   cmake_minimum_required(VERSION 3.8)
#   project(dbc_sygnal_mcm)
#   include(${CMAKE_CURRENT_SOURCE_DIR}/../cmake/dbc_package.cmake)
#   dbc_package(PREFIX mcm)
#
# PREFIX is required to keep identical stems in sibling subsystems apart, since the
# library name is also the C symbol prefix, C++ namespace and header directory:
#
#   mcm/Control.dbc -> mcm_control/mcm_control.hpp, namespace + symbols mcm_control
#
# This matches what the old cantools --database-name produced, so C symbols are
# unchanged across the migration.

# The single definition of how a DBC file maps to a library name. Used by dbc_package()
# and by the sygnal_dbc umbrella to emit matching compatibility headers.
function(dbc_library_name out_var prefix dbc_path)
  get_filename_component(_stem "${dbc_path}" NAME_WE)
  # cantools lowercases generated basenames; match it so capitalisation of the DBC
  # filename cannot shift the C symbol prefix.
  string(TOLOWER "${_stem}" _stem)
  set(${out_var} "${prefix}_${_stem}" PARENT_SCOPE)
endfunction()

macro(dbc_package)
  cmake_parse_arguments(_dbc_pkg "" "PREFIX" "" ${ARGN})

  if(NOT _dbc_pkg_PREFIX)
    message(FATAL_ERROR "dbc_package(${PROJECT_NAME}): PREFIX is required; it keeps "
      "identically-named DBCs in sibling subsystems from colliding.")
  endif()

  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
  endif()

  find_package(ament_cmake REQUIRED)
  # Provides the generate_dbc_cpp() macro.
  find_package(dbc_gen_cpp REQUIRED)

  # CONFIGURE_DEPENDS re-globs on rebuild, so a newly added .dbc is picked up
  # without a manual re-configure.
  file(GLOB _dbc_files CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.dbc")

  if(NOT _dbc_files)
    message(FATAL_ERROR "dbc_package(${PROJECT_NAME}): no .dbc files found in ${CMAKE_CURRENT_SOURCE_DIR}")
  endif()

  foreach(_dbc ${_dbc_files})
    dbc_library_name(_lib_name "${_dbc_pkg_PREFIX}" "${_dbc}")

    if(NOT _lib_name MATCHES "^[a-z_][a-z0-9_]*$")
      message(FATAL_ERROR
        "dbc_package(${PROJECT_NAME}): '${_lib_name}' (from ${_dbc}) is not a valid lowercase C/C++ "
        "identifier. Rename the DBC file to a snake_case name not starting with a digit.")
    endif()

    # Installs + exports itself, so downstream links ${PROJECT_NAME}::<lib_name>.
    generate_dbc_cpp(${_lib_name} DBC ${_dbc})
  endforeach()

  # Keep the raw DBCs available at runtime, as the single sygnal_dbc package used to.
  install(FILES ${_dbc_files} DESTINATION share/${PROJECT_NAME}/database)

  ament_package()
endmacro()
