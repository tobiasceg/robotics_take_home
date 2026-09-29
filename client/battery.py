"""Mock battery: the assignment allows batteryState to be faked."""
from __future__ import annotations


class MockBattery:
    """Drains slowly while driving, very slowly while idle. Never charges."""

    def __init__(
        self,
        charge: float = 100.0,
        drive_drain_per_s: float = 0.05,
        idle_drain_per_s: float = 0.005,
        floor: float = 5.0,
    ) -> None:
        self.charge = charge
        self._drive_drain = drive_drain_per_s
        self._idle_drain = idle_drain_per_s
        self._floor = floor

    def update(self, driving: bool, dt_s: float) -> float:
        drain = self._drive_drain if driving else self._idle_drain
        self.charge = max(self._floor, self.charge - drain * dt_s)
        return self.charge
