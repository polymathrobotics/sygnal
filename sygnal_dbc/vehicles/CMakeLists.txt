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

project(dbc_sygnal_vehicles VERSION 0.1.0 LANGUAGES C CXX)

# Auto-generate + export a CAN message library for every *.dbc in this directory.
include(${CMAKE_CURRENT_SOURCE_DIR}/../cmake/dbc_package.cmake)
dbc_package(PREFIX vehicles)
