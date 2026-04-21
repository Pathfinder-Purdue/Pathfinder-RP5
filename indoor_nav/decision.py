"""Decision engine: fused sector risks to navigation commands.

Pipeline:
  1. Score three candidate directions (LEFT, FORWARD, RIGHT) from fused [L,C,R] risks.
  2. Sort safest-first and iterate candidates.
  3. First candidate below thresholds becomes the command.
  4. If no direction is safe → STOP.
"""

import time
from dataclasses import dataclass

from indoor_nav.config import (
    STOP_THRESHOLD,
    CAUTION_THRESHOLD,
    HYSTERESIS,
    MIN_DWELL_MS,
)


@dataclass
class Decision:
    """Single nav command from the FSM."""
    command: str        # GO | SLOW | VEER LEFT | VEER RIGHT | STOP
    urgency: float      # 0.0 (safe) to 1.0 (critical)
    explanation: str    # why this command was picked


STATE_COMMANDS = {
    "CRUISE":      ("GO",         "Path is clear"),
    "CAUTION":     ("SLOW",       "Obstacle nearby - proceed with caution"),
    "AVOID_LEFT":  ("VEER LEFT",  "Safer to veer left"),
    "AVOID_RIGHT": ("VEER RIGHT", "Safer to veer right"),
    "STOPPED":     ("STOP",       "No safe direction found"),
}


class NavigationFSM:
    """Directional FSM based on fused [L, C, R] risks."""

    def __init__(self):
        self.state = "CRUISE"
        self._last_change_time = 0.0

    def update(self, risks):
        """Feed [L,C,R] fused risks.

        Returns a Decision.
        """
        candidate = self._pick_direction(risks)
        self._apply_with_dwell_gate(candidate)
        return self._build_decision(risks)

    # direction selection
    def _pick_direction(self, risks):
        """Score LEFT / FORWARD / RIGHT, return best FSM state."""
        left, center, right = risks

        # candidate directions sorted safest first
        candidates = [
            ("AVOID_LEFT",  left),
            ("CRUISE",      center),
            ("AVOID_RIGHT", right),
        ]
        candidates.sort(key=lambda x: x[1])

        for state, risk in candidates:
            # skip directions that are too risky
            if risk >= self._eff_threshold("STOPPED", STOP_THRESHOLD):
                continue

            # direction is viable
            if state == "CRUISE":
                if risk >= self._eff_threshold("CAUTION", CAUTION_THRESHOLD):
                    return "CAUTION"
                return "CRUISE"
            return state

        return "STOPPED"

    # thresholds and state machine
    def _eff_threshold(self, state_prefix, base):
        """Lower threshold by HYSTERESIS if already in this state."""
        if self.state.startswith(state_prefix):
            return base - HYSTERESIS
        return base

    def _apply_with_dwell_gate(self, candidate):
        """Block rapid state changes unless STOP (always immediate)."""
        if candidate == self.state:
            return
        now = time.time()
        elapsed_ms = (now - self._last_change_time) * 1000
        if candidate == "STOPPED" or elapsed_ms >= MIN_DWELL_MS:
            self.state = candidate
            self._last_change_time = now

    def _build_decision(self, risks):
        """Turn current FSM state into a Decision."""
        command, explanation = STATE_COMMANDS[self.state]
        urgency = max(risks)
        return Decision(command, urgency, explanation)
