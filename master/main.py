"""Entry point for the VDA5050 master.

Run from the repo root (WSL or anywhere with Python + paho-mqtt):
    python3 -m master.main order orders/pickup_dropoff.json   # send, then follow it to the end
    python3 -m master.main watch                              # just print what the robot is doing

Exit codes: 0 order complete, 1 order rejected or failed, 2 robot unreachable or timeout.
"""
from __future__ import annotations

import argparse
import json
import queue
import sys
import time
from datetime import datetime
from pathlib import Path

from master.link import MasterLink
from master.monitor import OrderMonitor
from vda5050 import topics
from vda5050.messages import HeaderFactory
from vda5050.schema import validation_errors
from vda5050.topics import AgvId

ACCEPT_TIMEOUT_S = 10.0


def say(line: str) -> None:
    print(f"{datetime.now():%H:%M:%S}  {line}", flush=True)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="VDA5050 master (fleet manager stand-in)")
    p.add_argument("--host", default="localhost", help="MQTT broker host")
    p.add_argument("--port", type=int, default=1883)
    p.add_argument("--manufacturer", default="robotis")
    p.add_argument("--serial", default="tb3_waffle_01", help="serialNumber of the robot to command")
    sub = p.add_subparsers(dest="command", required=True)

    order = sub.add_parser("order", help="send an order file and follow it until done")
    order.add_argument("file", type=Path)
    order.add_argument("--order-id", help="orderId to use (default: file's orderId + time, unique per send)")
    order.add_argument("--timeout", type=float, default=600.0, help="give up after this many seconds")

    sub.add_parser("watch", help="print the robot's connection and state changes until Ctrl+C")
    return p.parse_args()


def prepare_order(template: dict, headers: HeaderFactory, order_id: str) -> dict:
    """Fresh header (headerId, timestamp, manufacturer, serial) and orderId; the route is unchanged."""
    return {**template, **headers.next(topics.ORDER), "orderId": order_id, "orderUpdateId": 0}


def wait_for(link: MasterLink, monitor: OrderMonitor, topic_name: str, timeout: float) -> dict | None:
    """Process messages until one on `topic_name` arrives; print events on the way."""
    deadline = time.monotonic() + timeout
    while (remaining := deadline - time.monotonic()) > 0:
        try:
            name, msg = link.inbox.get(timeout=remaining)
        except queue.Empty:
            return None
        handle(monitor, name, msg)
        if name == topic_name:
            return msg
    return None


def handle(monitor: OrderMonitor, name: str, msg: dict) -> None:
    events = monitor.on_state(msg) if name == topics.STATE else monitor.on_connection(msg)
    for event in events:
        say(event)


def run_order(link: MasterLink, headers: HeaderFactory, args: argparse.Namespace) -> int:
    template = json.loads(args.file.read_text())
    order_id = args.order_id or f"{template['orderId']}-{datetime.now():%H%M%S}"
    order = prepare_order(template, headers, order_id)
    problems = validation_errors(topics.ORDER, order)
    if problems:
        say(f"not sending: order file is invalid: {problems[0]}")
        return 1

    monitor = OrderMonitor(order_id)
    connection = wait_for(link, monitor, topics.CONNECTION, timeout=3.0)
    if connection is None or connection["connectionState"] != "ONLINE":
        say(f"robot {args.serial} is not online ({connection['connectionState'] if connection else 'no status'})")
        return 2
    first_state = wait_for(link, monitor, topics.STATE, timeout=5.0)
    if first_state is None:
        say("robot is online but not publishing state")
        return 2
    monitor.baseline(first_state)

    nodes = [n["nodeId"] for n in order["nodes"]]
    actions = [f"{a['actionType']} at {n['nodeId']}" for n in order["nodes"] for a in n["actions"]]
    link.publish(topics.ORDER, order)
    say(f"sent order {order_id}: {' -> '.join(nodes)} ({', '.join(actions)})")

    sent_at = time.monotonic()
    while not monitor.done and monitor.rejection is None:
        if not monitor.accepted and time.monotonic() - sent_at > ACCEPT_TIMEOUT_S:
            say(f"robot did not accept the order within {ACCEPT_TIMEOUT_S:.0f}s")
            return 2
        remaining = args.timeout - (time.monotonic() - sent_at)
        if wait_for(link, monitor, topics.STATE, timeout=remaining) is None:
            say(f"timed out after {args.timeout:.0f}s")
            return 2
    if monitor.rejection:
        say(f"order {order_id} REJECTED: {monitor.rejection}")
        return 1
    return 0 if monitor.succeeded else 1


def run_watch(link: MasterLink) -> int:
    monitor = OrderMonitor()
    say("watching; Ctrl+C to stop")
    while True:
        name, msg = link.inbox.get()
        handle(monitor, name, msg)


def main() -> None:
    args = parse_args()
    agv = AgvId(args.manufacturer, args.serial)
    link = MasterLink(agv, args.host, args.port)
    link.start()
    try:
        code = run_order(link, HeaderFactory(agv), args) if args.command == "order" else run_watch(link)
    except KeyboardInterrupt:
        code = 0
    finally:
        link.stop()
    sys.exit(code)


if __name__ == "__main__":
    main()
