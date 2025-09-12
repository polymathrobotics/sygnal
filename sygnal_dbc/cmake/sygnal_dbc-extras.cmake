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
# sygnal_dbc extras included by ament's find_package

# Determine the installed share directory from the location of this config
get_filename_component(_pkg_share_dir "${sygnal_dbc_DIR}/.." ABSOLUTE)

# Public variable for consumers to locate the DBC database
set(SYGNAL_DBC_DIR "${_pkg_share_dir}/database")

if(NOT sygnal_dbc_FIND_QUIETLY)
  message(STATUS "sygnal_dbc: SYGNAL_DBC_DIR='${SYGNAL_DBC_DIR}'")
endif()
