"""Everything the client says to, and hears from, the MQTT broker.

Liveness (section 6.14):
  * Before connecting, register a last will: connection = CONNECTIONBROKEN.
    If we vanish without a clean disconnect, the broker publishes it for us.
  * After every (re)connect, publish connection = ONLINE.
  * On a clean shutdown, publish connection = OFFLINE, then disconnect.
All connection messages use QoS 1 and are retained, so a master that starts
later still sees the robot's latest status. Everything else uses QoS 0.
"""
from __future__ import annotations

import json
import logging
from typing import Callable

import paho.mqtt.client as mqtt

from vda5050 import topics
from vda5050.messages import ConnectionState, HeaderFactory, connection_message
from vda5050.topics import AgvId

log = logging.getLogger(__name__)

MessageHandler = Callable[[dict], None]


class MqttLink:
    def __init__(
        self,
        agv: AgvId,
        headers: HeaderFactory,
        handlers: dict[str, MessageHandler],
        host: str = "localhost",
        port: int = 1883,
        keepalive_s: int = 10,
    ) -> None:
        """`handlers` maps a topic name (e.g. "order") to the function that receives it."""
        self._agv = agv
        self._headers = headers
        self._handlers = handlers
        self._host, self._port, self._keepalive = host, port, keepalive_s

        self._client = mqtt.Client(
            mqtt.CallbackAPIVersion.VERSION2,
            client_id=f"vda5050-client-{agv.serial_number}",
        )
        self._client.on_connect = self._on_connect
        self._client.on_message = self._on_message

    def start(self) -> None:
        will = connection_message(self._headers.next(topics.CONNECTION), ConnectionState.CONNECTIONBROKEN)
        self._client.will_set(self._agv.topic(topics.CONNECTION), json.dumps(will), qos=1, retain=True)
        self._client.connect(self._host, self._port, keepalive=self._keepalive)
        self._client.loop_start()  # network traffic runs on paho's own thread

    def stop(self) -> None:
        info = self._publish_connection(ConnectionState.OFFLINE)
        info.wait_for_publish(timeout=2.0)
        self._client.disconnect()  # a clean disconnect means the broker discards the will
        self._client.loop_stop()

    def publish_state(self, state: dict) -> None:
        self._client.publish(self._agv.topic(topics.STATE), json.dumps(state), qos=0)

    def _publish_connection(self, state: ConnectionState) -> mqtt.MQTTMessageInfo:
        msg = connection_message(self._headers.next(topics.CONNECTION), state)
        return self._client.publish(self._agv.topic(topics.CONNECTION), json.dumps(msg), qos=1, retain=True)

    def _on_connect(self, client, userdata, flags, reason_code, properties) -> None:
        if reason_code.is_failure:
            log.error("MQTT connect failed: %s", reason_code)
            return
        log.info("connected to MQTT broker %s:%s", self._host, self._port)
        self._publish_connection(ConnectionState.ONLINE)
        for name in self._handlers:
            client.subscribe(self._agv.topic(name), qos=0)
            log.info("subscribed to %s", self._agv.topic(name))

    def _on_message(self, client, userdata, message: mqtt.MQTTMessage) -> None:
        name = message.topic.rsplit("/", 1)[-1]
        try:
            payload = json.loads(message.payload)
        except json.JSONDecodeError:
            log.warning("ignoring non-JSON message on %s", message.topic)
            return
        handler = self._handlers.get(name)
        if handler is not None:
            handler(payload)
