"""Turns the robot's stream of state messages into readable events.

Pure logic, no MQTT: feed it state/connection messages, get back lines like
"reached n4 (4 nodes left)" or "pick-1 (pick) FINISHED", and ask whether the
tracked order was accepted, rejected, completed or failed.
"""
from __future__ import annotations

TERMINAL = {"FINISHED", "FAILED"}
REJECTION_ERRORS = {"validationError", "orderError"}


class OrderMonitor:
    def __init__(self, order_id: str | None = None) -> None:
        """order_id=None follows whatever order the robot is running (watch mode)."""
        self.order_id = order_id
        self.accepted = False
        self.rejection: str | None = None
        self.done = False
        self.succeeded = False
        self.connection: str | None = None
        self.paused = False
        self._prev: dict | None = None
        self._seen_errors: set[tuple[str, str]] = set()

    def baseline(self, state: dict) -> None:
        """Remember errors already present before we sent anything, so they aren't blamed on our order."""
        self._seen_errors |= {(e["errorType"], e.get("errorDescription", "")) for e in state["errors"]}

    def on_connection(self, msg: dict) -> list[str]:
        new = msg["connectionState"]
        changed, self.connection = new != self.connection, new
        return [f"robot connection: {new}"] if changed else []

    def on_state(self, state: dict) -> list[str]:
        events = self._new_errors(state)
        if self.order_id is None:                 # watch mode: follow the robot's current order
            if state["orderId"] and (self._prev is None or state["orderId"] != self._prev["orderId"]):
                events.append(f"robot is on order {state['orderId']}")
                self._prev = None
        elif state["orderId"] != self.order_id:
            return events                          # still reporting an older order
        elif not self.accepted:
            self.accepted = True
            events.append(f"order {self.order_id} accepted")

        if state["orderId"]:
            events += self._diff(self._prev, state)
            events += self._check_finished(state)
        self._prev = state
        self.paused = state.get("paused", False)
        return events

    def _new_errors(self, state: dict) -> list[str]:
        events = []
        for err in state["errors"]:
            key = (err["errorType"], err.get("errorDescription", ""))
            if key in self._seen_errors:
                continue
            self._seen_errors.add(key)
            events.append(f"ERROR {key[0]}: {key[1]}")
            refs = {r["referenceKey"]: r["referenceValue"] for r in err.get("errorReferences", [])}
            if (self.order_id and not self.accepted and err["errorType"] in REJECTION_ERRORS
                    and refs.get("orderId", self.order_id) == self.order_id):
                self.rejection = f"{key[0]}: {key[1]}"
        return events

    @staticmethod
    def _diff(prev: dict | None, state: dict) -> list[str]:
        events = []
        prev = prev or {"lastNodeId": None, "driving": None, "paused": False, "actionStates": []}
        if state["lastNodeId"] != prev["lastNodeId"] and state["lastNodeId"]:
            pos, left = state["agvPosition"], len(state["nodeStates"])
            events.append(f"reached {state['lastNodeId']} at ({pos['x']:.2f}, {pos['y']:.2f}), "
                          f"{left} node{'' if left == 1 else 's'} left")
        if state.get("paused") != prev.get("paused"):
            events.append("PAUSED" if state.get("paused") else "RESUMED")
        if state["driving"] != prev["driving"]:
            events.append("driving" if state["driving"] else "stopped")
        before = {a["actionId"]: a["actionStatus"] for a in prev["actionStates"]}
        for a in state["actionStates"]:
            if before.get(a["actionId"]) != a["actionStatus"]:
                events.append(f"{a['actionId']} ({a['actionType']}) {a['actionStatus']}")
        return events

    def _check_finished(self, state: dict) -> list[str]:
        if self.order_id is None or self.done:
            return []
        statuses = [a["actionStatus"] for a in state["actionStates"]]
        if state["nodeStates"] or state["driving"] or any(s not in TERMINAL for s in statuses):
            return []
        self.done = True
        self.succeeded = all(s == "FINISHED" for s in statuses) and not any(
            e["errorType"] == "navigationFailed" for e in state["errors"])
        return [f"order {self.order_id} {'COMPLETE' if self.succeeded else 'FAILED'} "
                f"at {state['lastNodeId']}, battery {state['batteryState']['batteryCharge']}%"]
