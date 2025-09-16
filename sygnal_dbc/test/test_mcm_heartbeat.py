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
"""
Pytest that validates decoding of the MCM Heartbeat message using the
installed/raw DBC from this package. Mirrors the example frame from the
workspace `decode_heartbeat.py` script.
"""

from pathlib import Path

import cantools
from ament_index_python.packages import get_package_share_directory


def test_decode_mcm_heartbeat_from_installed_dbc():
    # Locate the installed DBC via ament index
    # Prefer the installed DBC via ament index
    dbc_path = None
    try:
        share_dir = Path(get_package_share_directory("sygnal_dbc"))
        candidate = share_dir / "database/mcm/Heartbeat.dbc"
        if candidate.is_file():
            dbc_path = candidate
    except Exception:
        pass

    # Fallback to source tree copy (works in non-installed test runs)
    if dbc_path is None:
        src_root = Path(__file__).resolve().parents[1]
        candidate = src_root / "database/mcm/Heartbeat.dbc"
        if candidate.is_file():
            dbc_path = candidate

    assert dbc_path and dbc_path.is_file(), (
        "Could not locate Heartbeat.dbc via ament or source path"
    )

    db = cantools.database.load_file(str(dbc_path))

    # Frame from the prompt: ID 0x170 with payload bytes below
    frame_id = 0x170
    data = bytes.fromhex("03 00 00 00 00 CE 10 2D")

    msg = db.get_message_by_frame_id(frame_id)

    # Decode numerically (no choice-to-string mapping) for stable assertions
    decoded = msg.decode(data, decode_choices=False)

    # Expectations derived from DBC bit layout (big-endian)
    # Byte layout: 03 00 00 00 00 CE 10 2D
    assert decoded["BusAddress"] == 3
    assert decoded["SubsystemID"] == 0

    assert decoded["SystemState"] == 0
    assert decoded["OverallInterfaceState"] == 0
    assert decoded["Interface0State"] == 0
    assert decoded["Interface1State"] == 0
    assert decoded["Interface2State"] == 0
    assert decoded["Interface3State"] == 0
    assert decoded["Interface4State"] == 0
    assert decoded["Interface5State"] == 0
    assert decoded["Interface6State"] == 0

    # Note: cantools decodes this 16-bit field as 0x10CE for this DBC layout
    assert decoded["Count16"] == 0x10CE
    assert decoded["CRC"] == 0x2D

    # Additionally validate that choice decoding maps SystemState 0
    # to the expected textual label in the DBC
    decoded_choices = msg.decode(data, decode_choices=True)
    assert decoded_choices["SystemState"] == "Human Control"
