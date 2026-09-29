"""Entry point for the VDA5050 client.

Run from the repo root inside WSL:
    python3 -m client.main

Milestone 3, step 4: MQTT side only. The robot reports an idle state and
validates incoming orders. Driving is added in steps 5 and 6.
"""
from __future__ import annotations

import argparse
import logging
import signal
import threading

from client.mqtt_link import MqttLink
from vda5050 import topics
from vda5050.messages import HeaderFactory, state_message
from vda5050.schema import validation_errors
from vda5050.topics import AgvId

log = logging.getLogger("client")


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="VDA5050 client for a simulated TurtleBot3")
    p.add_argument("--host", default="localhost", help="MQTT broker host")
    p.add_argument("--port", type=int, default=1883, help="MQTT broker port")
    p.add_argument("--manufacturer", default="robotis")
    p.add_argument("--serial", default="tb3_waffle_01", help="serialNumber of this robot")
    p.add_argument("--map-id", default="house_map")
    p.add_argument("--state-period", type=float, default=1.0, help="seconds between state messages")
    p.add_argument("--keepalive", type=int, default=10,
                   help="MQTT keepalive; the broker declares CONNECTIONBROKEN after ~1.5x this")
    return p.parse_args()


def on_order(msg: dict) -> None:
    problems = validation_errors(topics.ORDER, msg)
    if problems:
        log.warning("rejected invalid order: %s", "; ".join(problems[:3]))
        return
    node_ids = [n["nodeId"] for n in msg["nodes"]]
    log.info("received valid order %s: %s", msg["orderId"], " -> ".join(node_ids))


def idle_state(headers: HeaderFactory, map_id: str) -> dict:
    return state_message(
        headers.next(topics.STATE),
        order_id="", order_update_id=0, last_node_id="", last_node_sequence_id=0,
        node_states=[], edge_states=[], driving=False, paused=False,
        action_states=[], errors=[], position=None, battery_charge=100.0, map_id=map_id,
    )


def main() -> None:
    args = parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)-7s %(name)s: %(message)s")

    agv = AgvId(args.manufacturer, args.serial)
    headers = HeaderFactory(agv)
    link = MqttLink(agv, headers, handlers={topics.ORDER: on_order},
                    host=args.host, port=args.port, keepalive_s=args.keepalive)

    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())

    link.start()
    log.info("client %s running, publishing state every %.1fs", agv.serial_number, args.state_period)
    try:
        while not stop.wait(args.state_period):
            link.publish_state(idle_state(headers, args.map_id))
    finally:
        link.stop()
        log.info("client stopped cleanly")


if __name__ == "__main__":
    main()
