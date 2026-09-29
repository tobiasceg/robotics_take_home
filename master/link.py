"""The master's side of MQTT: publish order/instantActions, receive state/connection.

Incoming messages are put on `inbox` as (topic name, payload) so the main
thread handles them one at a time; paho's network thread never touches the
monitor.
"""
from __future__ import annotations

import json
import logging
import queue

import paho.mqtt.client as mqtt

from vda5050 import topics
from vda5050.topics import AgvId

log = logging.getLogger(__name__)


class MasterLink:
    def __init__(self, agv: AgvId, host: str = "localhost", port: int = 1883) -> None:
        self._agv = agv
        self._host, self._port = host, port
        self.inbox: queue.Queue[tuple[str, dict]] = queue.Queue()
        self._client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=f"vda5050-master-{agv.serial_number}")
        self._client.on_connect = self._on_connect
        self._client.on_message = self._on_message

    def start(self) -> None:
        self._client.connect(self._host, self._port)
        self._client.loop_start()

    def stop(self) -> None:
        self._client.disconnect()
        self._client.loop_stop()

    def publish(self, topic_name: str, msg: dict) -> None:
        """QoS 0 per spec for order and instantActions; wait until it has left this process."""
        info = self._client.publish(self._agv.topic(topic_name), json.dumps(msg), qos=0)
        info.wait_for_publish(timeout=2.0)

    def _on_connect(self, client, userdata, flags, reason_code, properties) -> None:
        if reason_code.is_failure:
            log.error("MQTT connect failed: %s", reason_code)
            return
        client.subscribe(self._agv.topic(topics.STATE), qos=0)
        client.subscribe(self._agv.topic(topics.CONNECTION), qos=1)  # retained: arrives at once

    def _on_message(self, client, userdata, message: mqtt.MQTTMessage) -> None:
        try:
            payload = json.loads(message.payload)
        except json.JSONDecodeError:
            log.warning("ignoring non-JSON message on %s", message.topic)
            return
        self.inbox.put((message.topic.rsplit("/", 1)[-1], payload))
