"""MQTT topic names (VDA5050 section 6.3).

Format: interfaceName/majorVersion/manufacturer/serialNumber/topic
Example: uagv/v2/robotis/tb3_waffle_01/order
"""
from __future__ import annotations

from dataclasses import dataclass

INTERFACE_NAME = "uagv"
MAJOR_VERSION = "v2"
PROTOCOL_VERSION = "2.1.0"

ORDER = "order"
INSTANT_ACTIONS = "instantActions"
STATE = "state"
CONNECTION = "connection"


@dataclass(frozen=True)
class AgvId:
    """Identifies one vehicle. Appears in every topic and every header."""

    manufacturer: str
    serial_number: str

    def topic(self, name: str) -> str:
        return f"{INTERFACE_NAME}/{MAJOR_VERSION}/{self.manufacturer}/{self.serial_number}/{name}"
