"""The shared contract must only ever produce messages the official schemas accept."""
import json
from pathlib import Path

import pytest

from vda5050.messages import (
    ActionState, ActionStatus, AgvError, ConnectionState, HeaderFactory, Position,
    connection_message, state_message,
)
from vda5050.order import Order
from vda5050.schema import validation_errors
from vda5050.topics import AgvId

AGV = AgvId("robotis", "tb3_waffle_01")
ORDER_FILE = Path(__file__).resolve().parent.parent / "orders" / "pickup_dropoff.json"


def _state(headers: HeaderFactory, **overrides) -> dict:
    fields = dict(
        order_id="", order_update_id=0, last_node_id="", last_node_sequence_id=0,
        node_states=[], edge_states=[], driving=False, paused=False,
        action_states=[], errors=[], position=None, battery_charge=100.0, map_id="house_map",
    )
    fields.update(overrides)
    return state_message(headers.next("state"), **fields)


def test_topic_format():
    assert AGV.topic("order") == "uagv/v2/robotis/tb3_waffle_01/order"


def test_header_ids_count_per_topic():
    headers = HeaderFactory(AGV)
    assert [headers.next("state")["headerId"] for _ in range(3)] == [0, 1, 2]
    assert headers.next("connection")["headerId"] == 0


@pytest.mark.parametrize("state", list(ConnectionState))
def test_connection_message_is_schema_valid(state):
    msg = connection_message(HeaderFactory(AGV).next("connection"), state)
    assert validation_errors("connection", msg) == []


def test_idle_state_is_schema_valid():
    assert validation_errors("state", _state(HeaderFactory(AGV))) == []


def test_busy_state_is_schema_valid():
    msg = _state(
        HeaderFactory(AGV),
        order_id="order-001", last_node_id="n4", last_node_sequence_id=6, driving=True,
        node_states=[{"nodeId": "n5", "sequenceId": 8, "released": True}],
        edge_states=[{"edgeId": "n4_n5", "sequenceId": 7, "released": True}],
        action_states=[ActionState("pick-1", "pick", ActionStatus.FINISHED)],
        errors=[AgvError("navigationFailed", "Nav2 aborted", references={"nodeId": "n5"})],
        position=Position(1.0, 2.0, 0.5),
    )
    assert validation_errors("state", msg) == []
    assert msg["agvPosition"]["positionInitialized"] is True


def test_handcrafted_order_is_schema_valid_and_parses():
    msg = json.loads(ORDER_FILE.read_text())
    assert validation_errors("order", msg) == []

    order = Order.from_message(msg)
    actions = {a.action_type: n.node_id for n in order.nodes for a in n.actions}
    assert [n.node_id for n in order.nodes] == [f"n{i}" for i in range(1, 9)]
    assert actions == {"pick": "n4", "drop": "n8"}


def test_order_without_node_position_is_rejected():
    msg = json.loads(ORDER_FILE.read_text())
    del msg["nodes"][2]["nodePosition"]
    with pytest.raises(ValueError, match="n3"):
        Order.from_message(msg)
