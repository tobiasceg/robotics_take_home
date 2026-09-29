"""Order executor: walks an order node by node and runs the mocked pick/drop.

Pure logic, no MQTT and no ROS. It is fed orders via submit(), advanced by
tick() from the main loop, and asked for its current state via state_fields().

    IDLE --order accepted--> DRIVING to next node --arrived--> node traversed
                                  ^                               |
                                  |                   node has pick/drop?
                                  |                     no  |      | yes
                                  +---- next node <---------+   ACTING: RUNNING,
                                  |                             wait, FINISHED
                                  +---------------------------------+
    last node traversed and its actions done --> IDLE (order complete)
    Nav2 fails --> error reported, remaining actions FAILED --> IDLE

Spec rules followed (VDA5050 6.6, 6.10.2, 6.11):
  * a node counts as traversed when Nav2 reports SUCCEEDED; it is removed from
    nodeStates, the edge leading to it from edgeStates, lastNodeId is set,
    and only then are the node's actions triggered
  * actionStates lists every action of the order, starting as WAITING
  * a new order is rejected (orderError) while the current one is running
  * an order with the same orderId + orderUpdateId as the one on the vehicle
    is discarded, even after it finished (so every dispatch needs a new orderId)
  * rejection errors stay in the state until a new order is accepted
"""
from __future__ import annotations

import logging
import time
from collections import deque
from enum import Enum
from typing import Callable

from client.navigator import Navigator, NavResult
from vda5050.messages import ActionState, ActionStatus, AgvError
from vda5050.order import Node, Order
from vda5050.schema import validation_errors

log = logging.getLogger(__name__)

SUPPORTED_ACTIONS = {"pick", "drop"}


class Phase(Enum):
    IDLE = "IDLE"
    DRIVING = "DRIVING"
    ACTING = "ACTING"


class OrderExecutor:
    def __init__(
        self,
        navigator: Navigator,
        action_duration_s: float = 2.0,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self._nav = navigator
        self._action_duration = action_duration_s
        self._clock = clock

        self._phase = Phase.IDLE
        self._order: Order | None = None
        self._traversed = 0                      # how many nodes of the order are done
        self._last_node_id = ""
        self._last_node_sequence_id = 0
        self._action_states: dict[str, ActionState] = {}
        self._pending_actions: deque[ActionState] = deque()
        self._current_action: ActionState | None = None
        self._action_deadline = 0.0
        self._errors: list[AgvError] = []
        self._changed = False

    # ---- inputs -----------------------------------------------------------------

    def submit(self, msg: dict) -> None:
        """Handle an order message. Call from the main loop thread only."""
        self._changed = True
        problems = validation_errors("order", msg)
        if problems:
            self._reject("validationError", "; ".join(problems[:3]), {"topic": "order"})
            return
        try:
            order = Order.from_message(msg)
        except ValueError as exc:
            self._reject("orderError", str(exc), {"orderId": msg["orderId"]})
            return

        current = self._order
        if current and (order.order_id, order.order_update_id) == (current.order_id, current.order_update_id):
            # Spec 6.6 step 6: same orderId + orderUpdateId is already on the vehicle -> discard,
            # whether it is still running or already finished.
            log.info("ignoring order %s: already on the vehicle", order.order_id)
            return
        if self._phase is not Phase.IDLE:
            self._reject("orderError", f"busy with order {current.order_id}; order updates are not supported",
                         {"orderId": order.order_id})
            return

        route_problem = _route_problem(order)
        if route_problem:
            self._reject("orderError", route_problem, {"orderId": order.order_id})
            return

        self._accept(order)

    def tick(self) -> bool:
        """Advance the order. Returns True if anything reportable changed."""
        if self._phase is Phase.DRIVING:
            result = self._nav.poll()
            if result is NavResult.SUCCEEDED:
                self._traverse(self._order.nodes[self._traversed])
            elif result is not None:
                self._abort(self._order.nodes[self._traversed], result)
        elif self._phase is Phase.ACTING and self._clock() >= self._action_deadline:
            self._current_action.action_status = ActionStatus.FINISHED
            log.info("action %s (%s) FINISHED", self._current_action.action_id, self._current_action.action_type)
            self._current_action = None
            self._changed = True
            self._run_next_action_or_drive()

        changed, self._changed = self._changed, False
        return changed

    # ---- outputs ----------------------------------------------------------------

    @property
    def driving(self) -> bool:
        return self._phase is Phase.DRIVING

    def state_fields(self) -> dict:
        """Everything state_message() needs except header, position, battery and mapId."""
        order, active = self._order, self._phase is not Phase.IDLE
        remaining_nodes = order.nodes[self._traversed:] if active else ()
        remaining_edges = order.edges[max(self._traversed - 1, 0):] if active else ()
        return {
            "order_id": order.order_id if order else "",
            "order_update_id": order.order_update_id if order else 0,
            "last_node_id": self._last_node_id,
            "last_node_sequence_id": self._last_node_sequence_id,
            "node_states": [
                {"nodeId": n.node_id, "sequenceId": n.sequence_id, "released": True} for n in remaining_nodes
            ],
            "edge_states": [
                {"edgeId": e.edge_id, "sequenceId": e.sequence_id, "released": True} for e in remaining_edges
            ],
            "driving": self.driving,
            "paused": False,
            "action_states": list(self._action_states.values()),
            "errors": list(self._errors),
        }

    # ---- internals --------------------------------------------------------------

    def _accept(self, order: Order) -> None:
        log.info("accepted order %s: %s", order.order_id, " -> ".join(n.node_id for n in order.nodes))
        self._order = order
        self._traversed = 0
        self._errors = []
        self._action_states = {
            a.action_id: ActionState(a.action_id, a.action_type) for n in order.nodes for a in n.actions
        }
        self._drive_to(order.nodes[0])

    def _reject(self, error_type: str, description: str, references: dict[str, str]) -> None:
        log.warning("order rejected (%s): %s", error_type, description)
        self._errors.append(AgvError(error_type, description, references=references))

    def _drive_to(self, node: Node) -> None:
        log.info("driving to %s (%.2f, %.2f)", node.node_id, node.x, node.y)
        self._phase = Phase.DRIVING
        self._nav.go_to(node.x, node.y, node.theta)

    def _traverse(self, node: Node) -> None:
        log.info("reached %s", node.node_id)
        self._traversed += 1
        self._last_node_id, self._last_node_sequence_id = node.node_id, node.sequence_id
        self._pending_actions = deque(self._action_states[a.action_id] for a in node.actions)
        self._changed = True
        self._run_next_action_or_drive()

    def _run_next_action_or_drive(self) -> None:
        while self._pending_actions:
            action = self._pending_actions.popleft()
            if action.action_type not in SUPPORTED_ACTIONS:
                action.action_status = ActionStatus.FAILED
                action.result_description = "unsupported action type"
                log.warning("action %s (%s) FAILED: unsupported", action.action_id, action.action_type)
                continue
            action.action_status = ActionStatus.RUNNING
            self._current_action = action
            self._action_deadline = self._clock() + self._action_duration
            self._phase = Phase.ACTING
            log.info("action %s (%s) RUNNING, mocked: waiting %.1fs",
                     action.action_id, action.action_type, self._action_duration)
            return

        if self._traversed < len(self._order.nodes):
            self._drive_to(self._order.nodes[self._traversed])
        else:
            self._phase = Phase.IDLE
            log.info("order %s complete", self._order.order_id)

    def _abort(self, node: Node, result: NavResult) -> None:
        log.error("navigation to %s %s; aborting order %s", node.node_id, result.value, self._order.order_id)
        self._errors.append(AgvError(
            "navigationFailed",
            f"Nav2 goal to node {node.node_id} ended with {result.value}",
            references={"orderId": self._order.order_id, "nodeId": node.node_id},
        ))
        for action in self._action_states.values():
            if action.action_status not in (ActionStatus.FINISHED, ActionStatus.FAILED):
                action.action_status = ActionStatus.FAILED
                action.result_description = "order aborted"
        self._pending_actions.clear()
        self._phase = Phase.IDLE
        self._changed = True


def _route_problem(order: Order) -> str | None:
    """This client drives one straight chain: node0 -edge0- node1 -edge1- node2 ..."""
    nodes, edges = order.nodes, order.edges
    if len(edges) != len(nodes) - 1:
        return f"expected {len(nodes) - 1} edges for {len(nodes)} nodes, got {len(edges)}"
    for i, edge in enumerate(edges):
        if (edge.start_node_id, edge.end_node_id) != (nodes[i].node_id, nodes[i + 1].node_id):
            return f"edge {edge.edge_id} does not connect {nodes[i].node_id} -> {nodes[i + 1].node_id}"
    return None
