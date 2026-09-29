"""Order executor behaviour, driven by the ROS-free SimulatedNavigator and a fake clock."""
import copy
import json
from pathlib import Path

import pytest

from client.executor import OrderExecutor
from client.navigator import SimulatedNavigator
from vda5050.messages import HeaderFactory, Position, state_message
from vda5050.schema import validation_errors
from vda5050.topics import AgvId

ORDER = json.loads((Path(__file__).resolve().parent.parent / "orders" / "pickup_dropoff.json").read_text())
N5 = next(n["nodePosition"] for n in ORDER["nodes"] if n["nodeId"] == "n5")


class FakeClock:
    def __init__(self) -> None:
        self.t = 0.0

    def __call__(self) -> float:
        return self.t


def make(fail_at=None):
    clock = FakeClock()
    nav = SimulatedNavigator(start=Position(-0.762, 2.365, 0.0), speed_mps=1.0, clock=clock, fail_at=fail_at)
    return OrderExecutor(nav, action_duration_s=2.0, clock=clock), clock


def run(executor, clock, seconds=120.0, dt=0.1):
    """Tick until idle-and-quiet or timeout; return a snapshot of state_fields() at every change."""
    snapshots = []
    for _ in range(int(seconds / dt)):
        clock.t += dt
        if executor.tick():
            snapshots.append(copy.deepcopy(executor.state_fields()))
    return snapshots


def statuses(fields):
    return {a.action_id: a.action_status.value for a in fields["action_states"]}


def test_full_order_picks_at_n4_then_drops_at_n8():
    ex, clock = make()
    ex.submit(ORDER)
    snaps = run(ex, clock)

    events = [(s["last_node_id"], statuses(s)["pick-1"], statuses(s)["drop-1"]) for s in snaps]
    assert ("n4", "RUNNING", "WAITING") in events
    assert ("n4", "FINISHED", "WAITING") in events
    assert ("n8", "FINISHED", "RUNNING") in events
    assert events.index(("n4", "FINISHED", "WAITING")) < events.index(("n8", "FINISHED", "RUNNING"))

    final = ex.state_fields()
    assert final["order_id"] == "order-001"
    assert final["last_node_id"] == "n8" and final["last_node_sequence_id"] == 14
    assert final["node_states"] == [] and final["edge_states"] == []
    assert final["driving"] is False and final["errors"] == []
    assert statuses(final) == {"pick-1": "FINISHED", "drop-1": "FINISHED"}


def test_robot_stands_still_while_picking():
    ex, clock = make()
    ex.submit(ORDER)
    picking = [s for s in run(ex, clock) if statuses(s)["pick-1"] == "RUNNING"]
    assert picking and all(not s["driving"] and s["last_node_id"] == "n4" for s in picking)


def test_node_and_edge_states_shrink_as_nodes_are_traversed():
    ex, clock = make()
    ex.submit(ORDER)
    at_n4 = next(s for s in run(ex, clock) if s["last_node_id"] == "n4")
    assert [n["nodeId"] for n in at_n4["node_states"]] == ["n5", "n6", "n7", "n8"]
    assert [e["edgeId"] for e in at_n4["edge_states"]] == ["n4_n5", "n5_n6", "n6_n7", "n7_n8"]


def test_navigation_failure_is_reported_and_frees_the_vehicle():
    ex, clock = make(fail_at={(N5["x"], N5["y"])})
    ex.submit(ORDER)
    run(ex, clock)

    final = ex.state_fields()
    assert [e.error_type for e in final["errors"]] == ["navigationFailed"]
    assert final["errors"][0].references["nodeId"] == "n5"
    assert statuses(final) == {"pick-1": "FINISHED", "drop-1": "FAILED"}
    assert final["node_states"] == [] and not final["driving"]

    retry = dict(ORDER, orderId="order-002")
    ex.submit(retry)                      # vehicle is free again, so this is accepted
    assert ex.state_fields()["order_id"] == "order-002" and ex.state_fields()["errors"] == []


def test_new_order_while_busy_is_rejected():
    ex, clock = make()
    ex.submit(ORDER)
    run(ex, clock, seconds=1.0)
    ex.submit(dict(ORDER, orderId="order-002"))

    fields = ex.state_fields()
    assert fields["order_id"] == "order-001" and fields["driving"]
    assert [e.error_type for e in fields["errors"]] == ["orderError"]


def test_resending_the_same_order_is_ignored():
    ex, clock = make()
    ex.submit(ORDER)
    run(ex, clock, seconds=1.0)
    ex.submit(ORDER)
    assert ex.state_fields()["errors"] == []


def test_finished_order_resent_with_same_id_is_ignored_but_new_id_runs():
    ex, clock = make()
    ex.submit(ORDER)
    run(ex, clock)
    ex.submit(ORDER)
    assert not ex.driving and ex.state_fields()["node_states"] == []

    ex.submit(dict(ORDER, orderId="order-002"))
    assert ex.driving and ex.state_fields()["order_id"] == "order-002"


def test_invalid_order_reports_validation_error():
    ex, _ = make()
    ex.submit({"orderId": "broken"})
    fields = ex.state_fields()
    assert [e.error_type for e in fields["errors"]] == ["validationError"]
    assert fields["order_id"] == "" and not fields["driving"]


def test_order_whose_edges_do_not_chain_the_nodes_is_rejected():
    bad = copy.deepcopy(ORDER)
    bad["edges"][2]["endNodeId"] = "n7"
    ex, _ = make()
    ex.submit(bad)
    assert [e.error_type for e in ex.state_fields()["errors"]] == ["orderError"]


@pytest.mark.parametrize("fail_at", [None, {(N5["x"], N5["y"])}])
def test_every_published_state_is_schema_valid(fail_at):
    ex, clock = make(fail_at)
    headers = HeaderFactory(AgvId("robotis", "tb3_waffle_01"))
    ex.submit(ORDER)
    for fields in run(ex, clock):
        msg = state_message(headers.next("state"), **fields,
                            position=Position(1.0, 2.0, 0.3), battery_charge=80.0, map_id="house_map")
        assert validation_errors("state", msg) == []
