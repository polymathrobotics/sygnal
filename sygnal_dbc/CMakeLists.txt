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

project(sygnal_dbc VERSION 0.1.0 LANGUAGES C)

find_package(ament_cmake REQUIRED)
find_package(Python3 COMPONENTS Interpreter REQUIRED)
include(GNUInstallDirs)

# Install the DBC database directory
install(
  DIRECTORY database/
  DESTINATION share/${PROJECT_NAME}/database
)


# Generate C sources/headers from all .dbc files under database/
file(GLOB_RECURSE DBC_FILES CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/database/*.dbc")

set(GENERATED_C_SOURCES)
set(GENERATED_HEADERS)

foreach(DBC_FILE IN LISTS DBC_FILES)
  # Compute path pieces to mirror the database/ layout under build dir
  file(RELATIVE_PATH REL_PATH "${CMAKE_CURRENT_SOURCE_DIR}/database" "${DBC_FILE}")
  get_filename_component(REL_DIR "${REL_PATH}" DIRECTORY)
  get_filename_component(DBC_BASENAME_WE "${DBC_FILE}" NAME_WE)

  # cantools generates lowercase file basenames; mirror that here
  string(TOLOWER "${DBC_BASENAME_WE}" MOD_NAME)

  # Generate directly into the build include tree so dependents
  # can include headers during the same colcon invocation.
  set(OUT_DIR "${CMAKE_CURRENT_BINARY_DIR}/include/${PROJECT_NAME}/${REL_DIR}")
  set(OUT_C   "${OUT_DIR}/${MOD_NAME}.c")
  set(OUT_H   "${OUT_DIR}/${MOD_NAME}.h")

  add_custom_command(
    OUTPUT ${OUT_C} ${OUT_H}
    COMMAND ${CMAKE_COMMAND} -E make_directory ${OUT_DIR}
    COMMAND ${Python3_EXECUTABLE} -m cantools generate_c_source ${DBC_FILE} -o ${OUT_DIR} --database-name ${MOD_NAME}
    DEPENDS ${DBC_FILE}
    COMMENT "Generating C source from DBC '${REL_PATH}' with cantools"
    VERBATIM
  )

  list(APPEND GENERATED_C_SOURCES ${OUT_C})
  list(APPEND GENERATED_HEADERS   ${OUT_H})
endforeach()

add_library(${PROJECT_NAME} STATIC ${GENERATED_C_SOURCES})
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Ensure generation runs as part of normal build (sources already bind it)
add_custom_target(generate_can_sources ALL
  DEPENDS ${GENERATED_C_SOURCES} ${GENERATED_HEADERS})

# Install generated headers so dependents can include them
install(
  DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Export a config extras so consumers get SYGNAL_DBC_DIR
set(_extras_file "${CMAKE_CURRENT_SOURCE_DIR}/cmake/${PROJECT_NAME}-extras.cmake")
set(_extras_args)
if(EXISTS ${_extras_file})
  set(_extras_args CONFIG_EXTRAS "cmake/${PROJECT_NAME}-extras.cmake")
endif()

# Export only modern CMake targets; include dirs and libs are already
# encoded on the exported target usage requirements above.
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package(${_extras_args})
