"""Entry point for the VDA5050 client.

Run from the repo root inside WSL, with Gazebo + Nav2 running and the robot's
pose set in RViz:
    python3 -m client.main

Without a simulator (straight-line fake robot, no ROS needed):
    python3 -m client.main --fake-nav

Threads: paho's network thread receives orders and instantActions and puts them on a queue; the
main loop below is the only thread that touches the executor. It ticks 10x a
second and publishes state on every change and at least every --state-period.
"""
from __future__ import annotations

import argparse
import logging
import queue
import signal
import threading
import time

from client.battery import MockBattery
from client.executor import OrderExecutor
from client.mqtt_link import MqttLink
from client.navigator import Navigator, SimulatedNavigator
from vda5050 import topics
from vda5050.messages import HeaderFactory, Position, state_message
from vda5050.topics import AgvId

log = logging.getLogger("client")

TICK_S = 0.1


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="VDA5050 client for a simulated TurtleBot3")
    p.add_argument("--host", default="localhost", help="MQTT broker host")
    p.add_argument("--port", type=int, default=1883, help="MQTT broker port")
    p.add_argument("--manufacturer", default="robotis")
    p.add_argument("--serial", default="tb3_waffle_01", help="serialNumber of this robot")
    p.add_argument("--map-id", default="house_map")
    p.add_argument("--state-period", type=float, default=1.0, help="max seconds between state messages")
    p.add_argument("--action-duration", type=float, default=2.0, help="seconds a mocked pick/drop takes")
    p.add_argument("--keepalive", type=int, default=10,
                   help="MQTT keepalive; the broker declares CONNECTIONBROKEN after ~1.5x this")
    p.add_argument("--fake-nav", action="store_true",
                   help="use a straight-line simulated robot instead of ROS 2 Nav2")
    return p.parse_args()


def make_navigator(fake: bool) -> Navigator:
    if fake:
        log.info("using SimulatedNavigator (no ROS)")
        return SimulatedNavigator(start=Position(-0.762, 2.365, -0.908))   # n1, the spawn point
    from client.nav2_navigator import Nav2Navigator   # imported here so --fake-nav needs no ROS
    log.info("using Nav2Navigator")
    return Nav2Navigator()


def main() -> None:
    args = parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)-7s %(name)s: %(message)s")

    agv = AgvId(args.manufacturer, args.serial)
    headers = HeaderFactory(agv)
    navigator = make_navigator(args.fake_nav)
    executor = OrderExecutor(navigator, action_duration_s=args.action_duration)
    battery = MockBattery()

    inbox: queue.Queue[tuple[str, dict]] = queue.Queue()
    handlers = {name: (lambda msg, name=name: inbox.put((name, msg)))
                for name in (topics.ORDER, topics.INSTANT_ACTIONS)}
    dispatch = {topics.ORDER: executor.submit, topics.INSTANT_ACTIONS: executor.submit_instant_actions}
    link = MqttLink(agv, headers, handlers=handlers,
                    host=args.host, port=args.port, keepalive_s=args.keepalive)

    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())

    link.start()
    log.info("client %s running", agv.serial_number)
    last_publish = 0.0
    try:
        while not stop.wait(TICK_S):
            while not inbox.empty():
                name, msg = inbox.get_nowait()
                dispatch[name](msg)
            changed = executor.tick()
            battery.update(executor.driving, TICK_S)

            now = time.monotonic()
            if changed or now - last_publish >= args.state_period:
                link.publish_state(state_message(
                    headers.next(topics.STATE),
                    **executor.state_fields(),
                    position=navigator.pose(),
                    battery_charge=battery.charge,
                    map_id=args.map_id,
                ))
                last_publish = now
    finally:
        link.stop()
        navigator.close()
        log.info("client stopped cleanly")


if __name__ == "__main__":
    main()
