"""Typed view of an incoming order (section 6.6), so the client never digs through raw dicts."""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Action:
    action_id: str
    action_type: str
    blocking_type: str


@dataclass(frozen=True)
class Node:
    node_id: str
    sequence_id: int
    x: float
    y: float
    theta: float | None
    map_id: str
    actions: tuple[Action, ...]


@dataclass(frozen=True)
class Edge:
    edge_id: str
    sequence_id: int
    start_node_id: str
    end_node_id: str


@dataclass(frozen=True)
class Order:
    order_id: str
    order_update_id: int
    nodes: tuple[Node, ...]
    edges: tuple[Edge, ...]

    @classmethod
    def from_message(cls, msg: dict) -> Order:
        """Parse a schema-valid order message.

        Raises ValueError for things the schema allows but this client cannot
        drive: a node without a position.
        """
        nodes = []
        for raw in sorted(msg["nodes"], key=lambda n: n["sequenceId"]):
            pos = raw.get("nodePosition")
            if pos is None:
                raise ValueError(f"node {raw['nodeId']} has no nodePosition")
            nodes.append(Node(
                node_id=raw["nodeId"],
                sequence_id=raw["sequenceId"],
                x=pos["x"],
                y=pos["y"],
                theta=pos.get("theta"),
                map_id=pos["mapId"],
                actions=tuple(
                    Action(a["actionId"], a["actionType"], a["blockingType"])
                    for a in raw["actions"]
                ),
            ))
        edges = tuple(
            Edge(e["edgeId"], e["sequenceId"], e["startNodeId"], e["endNodeId"])
            for e in sorted(msg["edges"], key=lambda e: e["sequenceId"])
        )
        return cls(msg["orderId"], msg["orderUpdateId"], tuple(nodes), edges)
