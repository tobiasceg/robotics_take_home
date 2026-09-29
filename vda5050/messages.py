"""Builders for outgoing VDA5050 messages: header, connection, state."""
from __future__ import annotations

import threading
from collections import defaultdict
from dataclasses import dataclass, field
from datetime import datetime, timezone
from enum import Enum

from vda5050.topics import PROTOCOL_VERSION, AgvId


class ConnectionState(str, Enum):
    ONLINE = "ONLINE"
    OFFLINE = "OFFLINE"                    # we disconnect on purpose
    CONNECTIONBROKEN = "CONNECTIONBROKEN"  # sent by the broker as our last will


class ActionStatus(str, Enum):
    WAITING = "WAITING"
    INITIALIZING = "INITIALIZING"
    RUNNING = "RUNNING"
    PAUSED = "PAUSED"
    FINISHED = "FINISHED"
    FAILED = "FAILED"


@dataclass(frozen=True)
class Position:
    """Robot pose in the map frame (metres, radians)."""

    x: float
    y: float
    theta: float


@dataclass
class ActionState:
    action_id: str
    action_type: str
    action_status: ActionStatus = ActionStatus.WAITING
    result_description: str = ""

    def to_dict(self) -> dict:
        out = {
            "actionId": self.action_id,
            "actionType": self.action_type,
            "actionStatus": self.action_status.value,
        }
        if self.result_description:
            out["resultDescription"] = self.result_description
        return out


@dataclass
class AgvError:
    error_type: str
    description: str
    error_level: str = "WARNING"          # WARNING or FATAL
    references: dict[str, str] = field(default_factory=dict)

    def to_dict(self) -> dict:
        return {
            "errorType": self.error_type,
            "errorLevel": self.error_level,
            "errorDescription": self.description,
            "errorReferences": [
                {"referenceKey": k, "referenceValue": v} for k, v in self.references.items()
            ],
        }


def utc_timestamp() -> str:
    """ISO 8601 UTC with milliseconds, e.g. 2026-09-29T08:15:30.123Z."""
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z")


class HeaderFactory:
    """Creates the protocol header (section 6.4).

    headerId is counted separately per topic and goes up by one for every
    message sent on that topic. Thread-safe, because MQTT callbacks and the
    main loop both send messages.
    """

    def __init__(self, agv: AgvId) -> None:
        self._agv = agv
        self._next_id: dict[str, int] = defaultdict(int)
        self._lock = threading.Lock()

    def next(self, topic_name: str) -> dict:
        with self._lock:
            header_id = self._next_id[topic_name]
            self._next_id[topic_name] += 1
        return {
            "headerId": header_id,
            "timestamp": utc_timestamp(),
            "version": PROTOCOL_VERSION,
            "manufacturer": self._agv.manufacturer,
            "serialNumber": self._agv.serial_number,
        }


def connection_message(header: dict, state: ConnectionState) -> dict:
    return {**header, "connectionState": state.value}


def state_message(
    header: dict,
    *,
    order_id: str,
    order_update_id: int,
    last_node_id: str,
    last_node_sequence_id: int,
    node_states: list[dict],
    edge_states: list[dict],
    driving: bool,
    paused: bool,
    action_states: list[ActionState],
    errors: list[AgvError],
    position: Position | None,
    battery_charge: float,
    map_id: str,
) -> dict:
    """Build a schema-valid state message (section 6.10.6).

    Fields the assignment asks for, plus the ones the official schema requires
    (nodeStates, edgeStates, operatingMode, safetyState, ...) with fixed
    values where this simulation has nothing real to report.
    """
    pose = position or Position(0.0, 0.0, 0.0)
    return {
        **header,
        "orderId": order_id,
        "orderUpdateId": order_update_id,
        "lastNodeId": last_node_id,
        "lastNodeSequenceId": last_node_sequence_id,
        "nodeStates": node_states,
        "edgeStates": edge_states,
        "agvPosition": {
            "x": round(pose.x, 3),
            "y": round(pose.y, 3),
            "theta": round(pose.theta, 4),
            "mapId": map_id,
            "positionInitialized": position is not None,
        },
        "driving": driving,
        "paused": paused,
        "actionStates": [a.to_dict() for a in action_states],
        "batteryState": {"batteryCharge": round(battery_charge, 1), "charging": False},
        "operatingMode": "AUTOMATIC",
        "errors": [e.to_dict() for e in errors],
        "safetyState": {"eStop": "NONE", "fieldViolation": False},
    }
