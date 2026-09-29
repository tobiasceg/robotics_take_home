"""The master's OrderMonitor, fed with real state messages produced by the client's executor.

This exercises both halves through the shared contract: executor -> state_message
-> OrderMonitor, exactly the path the messages take over MQTT.
"""
import json
from pathlib import Path

from client.executor import OrderExecutor
from client.navigator import SimulatedNavigator
from master.main import prepare_order
from master.monitor import OrderMonitor
from vda5050.messages import HeaderFactory, Position, state_message
from vda5050.topics import AgvId

AGV = AgvId("robotis", "tb3_waffle_01")
TEMPLATE = json.loads((Path(__file__).resolve().parent.parent / "orders" / "pickup_dropoff.json").read_text())
N5 = next(n["nodePosition"] for n in TEMPLATE["nodes"] if n["nodeId"] == "n5")


class FakeClock:
    def __init__(self) -> None:
        self.t = 0.0

    def __call__(self) -> float:
        return self.t


class Robot:
    """The client's executor plus a fake navigator, producing real state messages."""

    def __init__(self, fail_at=None):
        self.clock = FakeClock()
        self.nav = SimulatedNavigator(start=Position(-0.762, 2.365, 0.0), speed_mps=1.0,
                                      clock=self.clock, fail_at=fail_at)
        self.executor = OrderExecutor(self.nav, action_duration_s=2.0, clock=self.clock)
        self.headers = HeaderFactory(AGV)

    def state(self) -> dict:
        return state_message(self.headers.next("state"), **self.executor.state_fields(),
                             position=self.nav.pose(), battery_charge=90.0, map_id="house_map")

    def states(self, seconds=120.0, dt=0.1):
        for _ in range(int(seconds / dt)):
            self.clock.t += dt
            if self.executor.tick():
                yield self.state()


def follow(robot, monitor, order):
    monitor.baseline(robot.state())
    robot.executor.submit(order)
    events = monitor.on_state(robot.state())
    for s in robot.states():
        events += monitor.on_state(s)
        if monitor.done:
            break
    return events


def test_master_sees_pick_at_n4_then_drop_at_n8_then_complete():
    order = prepare_order(TEMPLATE, HeaderFactory(AGV), "order-A")
    monitor = OrderMonitor("order-A")
    events = follow(Robot(), monitor, order)

    assert events[0] == "order order-A accepted"
    pick_done = events.index("pick-1 (pick) FINISHED")
    drop_run = events.index("drop-1 (drop) RUNNING")
    assert any(e.startswith("reached n4") for e in events[:pick_done])
    assert any(e.startswith("reached n8") for e in events[pick_done:drop_run])
    assert pick_done < drop_run
    assert monitor.done and monitor.succeeded
    assert events[-1].startswith("order order-A COMPLETE at n8")


def test_master_reports_failure_when_navigation_fails():
    monitor = OrderMonitor("order-B")
    events = follow(Robot(fail_at={(N5["x"], N5["y"])}), monitor,
                    prepare_order(TEMPLATE, HeaderFactory(AGV), "order-B"))
    assert any(e.startswith("ERROR navigationFailed") for e in events)
    assert monitor.done and not monitor.succeeded
    assert events[-1].startswith("order order-B FAILED")


def test_master_detects_rejection_when_robot_is_busy():
    robot = Robot()
    robot.executor.submit(prepare_order(TEMPLATE, HeaderFactory(AGV), "order-first"))
    next(robot.states())

    monitor = OrderMonitor("order-second")
    monitor.baseline(robot.state())
    robot.executor.submit(prepare_order(TEMPLATE, HeaderFactory(AGV), "order-second"))
    monitor.on_state(robot.state())
    assert not monitor.accepted
    assert monitor.rejection.startswith("orderError: busy with order order-first")


def test_old_errors_are_not_blamed_on_a_new_order():
    robot = Robot()
    robot.executor.submit({"orderId": "garbage"})          # leaves a validationError behind
    monitor = OrderMonitor("order-C")
    monitor.baseline(robot.state())
    robot.executor.submit(prepare_order(TEMPLATE, HeaderFactory(AGV), "order-C"))
    monitor.on_state(robot.state())
    assert monitor.accepted and monitor.rejection is None


def test_prepared_order_has_fresh_header_and_new_id_but_same_route():
    headers = HeaderFactory(AGV)
    first, second = (prepare_order(TEMPLATE, headers, i) for i in ("x-1", "x-2"))
    assert (first["headerId"], second["headerId"]) == (0, 1)
    assert first["orderId"] == "x-1" and first["nodes"] == TEMPLATE["nodes"]
