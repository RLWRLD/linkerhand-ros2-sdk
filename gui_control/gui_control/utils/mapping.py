"""Compatibility wrapper exposing shared mapping data for the GUI.

The LinkerHand SDK already maintains the authoritative joint angle mappings.
Re-export everything from that module so both the SDK nodes and the GUI use
identical conversion tables.
"""

from linker_hand_ros2_sdk.LinkerHand.utils.mapping import *  # noqa: F401,F403
