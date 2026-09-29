"""startPause / stopPause instant actions (Milestone 5)."""
import json
from pathlib import Path

from client.executor import OrderExecutor
from client.navigator import SimulatedNavigator
from master.main import prepare_order
from master.monitor import OrderMonitor
from vda5050.messages import (
    HeaderFactory, Position, instant_action, instant_actions_message, state_message,
)
from vda5050.schema import validation_errors
from vda5050.topics import AgvId

AGV = AgvId("robotis", "tb3_waffle_01")
ORDER = json.loads((Path(__file__).resolve().parent.parent / "orders" / "pickup_dropoff.json").read_text())


class FakeClock:
    def __init__(self) -> None:
        self.t = 0.0

    def __call__(self) -> float:
        return self.t


class SpyNavigator(SimulatedNavigator):
    """Records every goal it is sent."""

    def __init__(self, **kw):
        super().__init__(**kw)
        self.goals = []

    def go_to(self, x, y, theta):
        self.goals.append((x, y))
        super().go_to(x, y, theta)


def make():
    clock = FakeClock()
    nav = SpyNavigator(start=Position(-0.762, 2.365, 0.0), speed_mps=1.0, clock=clock)
    return OrderExecutor(nav, action_duration_s=2.0, clock=clock), nav, clock


def advance(ex, clock, seconds, dt=0.1):
    for _ in range(round(seconds / dt)):
        clock.t += dt
        ex.tick()


def run_until(ex, clock, condition, limit=200.0, dt=0.1):
    for _ in range(int(limit / dt)):
        if condition(ex.state_fields()):
            return
        clock.t += dt
        ex.tick()
    raise AssertionError("condition never met")


MASTER_HEADERS = HeaderFactory(AGV)


def pause_msg(action_type, action_id):
    return instant_actions_message(MASTER_HEADERS.next("instantActions"), [instant_action(action_type, action_id)])


def status(ex, action_id):
    return next(a.action_status.value for a in ex.state_fields()["action_states"] if a.action_id == action_id)


def test_instant_actions_message_is_schema_valid():
    assert validation_errors("instantActions", pause_msg("startPause", "p-1")) == []


def test_pause_mid_edge_stops_and_resume_continues_to_the_same_node():
    ex, nav, clock = make()
    ex.submit(ORDER)
    advance(ex, clock, 0.8)                      # n1 reached at once; now between n1 and n2
    fields = ex.state_fields()
    assert fields["driving"] and fields["last_node_id"] == "n1"
    target_before = nav.goals[-1]

    ex.submit_instant_actions(pause_msg("startPause", "p-1"))
    assert status(ex, "p-1") == "RUNNING"        # cancel sent, not yet confirmed
    ex.tick()                                    # navigator reports CANCELED
    fields = ex.state_fields()
    assert fields["paused"] and not fields["driving"]
    assert status(ex, "p-1") == "FINISHED"
    assert fields["last_node_id"] == "n1"        # order kept, no progress lost
    assert [n["nodeId"] for n in fields["node_states"]][0] == "n2"

    frozen = nav.pose()
    advance(ex, clock, 10.0)
    assert nav.pose() == frozen and ex.state_fields()["paused"]   # really standing still

    ex.submit_instant_actions(pause_msg("stopPause", "r-1"))
    fields = ex.state_fields()
    assert not fields["paused"] and fields["driving"]
    assert status(ex, "r-1") == "FINISHED"
    assert nav.goals[-1] == target_before        # resumed toward the same node (n2)

    run_until(ex, clock, lambda f: f["last_node_id"] == "n8" and not f["driving"] and not f["node_states"]
              and all(a.action_status.value == "FINISHED" for a in f["action_states"]))


def test_pause_during_pick_pauses_the_action_and_keeps_its_remaining_time():
    ex, _, clock = make()
    ex.submit(ORDER)
    run_until(ex, clock, lambda f: f["last_node_id"] == "n4")
    advance(ex, clock, 0.5)                      # pick has 1.5 s left
    ex.submit_instant_actions(pause_msg("startPause", "p-2"))
    assert status(ex, "pick-1") == "PAUSED" and status(ex, "p-2") == "FINISHED"
    assert ex.state_fields()["paused"]

    advance(ex, clock, 30.0)
    assert status(ex, "pick-1") == "PAUSED"      # paused time does not count

    ex.submit_instant_actions(pause_msg("stopPause", "r-2"))
    assert status(ex, "pick-1") == "RUNNING"
    advance(ex, clock, 1.3)
    assert status(ex, "pick-1") == "RUNNING"
    advance(ex, clock, 0.4)
    assert status(ex, "pick-1") == "FINISHED" and ex.driving


def test_resume_before_the_cancel_lands_just_keeps_driving_to_the_same_node():
    ex, nav, clock = make()
    ex.submit(ORDER)
    advance(ex, clock, 0.8)
    target = nav.goals[-1]
    ex.submit_instant_actions(pause_msg("startPause", "p-3"))
    ex.submit_instant_actions(pause_msg("stopPause", "r-3"))   # before any tick sees CANCELED
    ex.tick()
    fields = ex.state_fields()
    assert not fields["paused"] and fields["driving"] and nav.goals[-1] == target
    assert status(ex, "p-3") == "FAILED" and status(ex, "r-3") == "FINISHED"


def test_pause_while_idle_holds_a_new_order_until_resume():
    ex, nav, clock = make()
    ex.submit_instant_actions(pause_msg("startPause", "p-4"))
    assert ex.state_fields()["paused"] and status(ex, "p-4") == "FINISHED"

    ex.submit(ORDER)
    advance(ex, clock, 5.0)
    fields = ex.state_fields()
    assert fields["order_id"] == "order-001" and fields["paused"] and not fields["driving"]
    assert nav.goals == []                        # not moved at all

    ex.submit_instant_actions(pause_msg("stopPause", "r-4"))
    advance(ex, clock, 0.2)
    assert ex.state_fields()["last_node_id"] == "n1" and ex.driving


def test_pause_twice_and_resume_when_not_paused_are_harmless():
    ex, _, clock = make()
    ex.submit(ORDER)
    advance(ex, clock, 0.8)
    ex.submit_instant_actions(pause_msg("stopPause", "r-5"))
    assert status(ex, "r-5") == "FINISHED" and ex.driving
    ex.submit_instant_actions(pause_msg("startPause", "p-5"))
    ex.tick()
    ex.submit_instant_actions(pause_msg("startPause", "p-6"))
    assert status(ex, "p-6") == "FINISHED" and ex.state_fields()["paused"]


def test_invalid_and_unsupported_instant_actions():
    ex, _, _ = make()
    ex.submit_instant_actions({"actions": "nope"})
    assert [e.error_type for e in ex.state_fields()["errors"]] == ["validationError"]
    ex.submit_instant_actions(pause_msg("startCharging", "c-1"))
    assert status(ex, "c-1") == "FAILED"


def test_real_cancel_from_elsewhere_still_aborts_the_order():
    """A CANCELED we did not ask for (e.g. someone cancels in RViz) is an abort, not a pause."""
    ex, nav, clock = make()
    ex.submit(ORDER)
    advance(ex, clock, 0.8)
    nav.cancel()
    ex.tick()
    fields = ex.state_fields()
    assert [e.error_type for e in fields["errors"]] == ["navigationFailed"]
    assert not fields["paused"] and not fields["node_states"]


def test_states_during_pause_are_schema_valid_and_master_sees_paused_resumed():
    ex, nav, clock = make()
    headers = HeaderFactory(AGV)
    monitor = OrderMonitor("order-P")

    def state():
        return state_message(headers.next("state"), **ex.state_fields(), position=nav.pose(),
                             battery_charge=90.0, map_id="house_map")

    monitor.baseline(state())
    ex.submit(prepare_order(ORDER, HeaderFactory(AGV), "order-P"))
    events = []
    for step in range(3000):
        if step == 8:
            ex.submit_instant_actions(pause_msg("startPause", "p-7"))
        if step == 60:
            ex.submit_instant_actions(pause_msg("stopPause", "r-7"))
        clock.t += 0.1
        ex.tick()
        msg = state()
        assert validation_errors("state", msg) == []
        events += monitor.on_state(msg)
        if monitor.done:
            break

    assert "p-7 (startPause) FINISHED" in events and "PAUSED" in events
    assert events.index("PAUSED") < events.index("RESUMED")
    assert monitor.done and monitor.succeeded
