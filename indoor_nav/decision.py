"""FSM that turns sector risk into nav commands: GO, SLOW, VEER LEFT, VEER RIGHT, STOP."""

import time
from dataclasses import dataclass

from indoor_nav.config import (
    STOP_THRESHOLD,
    AVOID_THRESHOLD,
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


# maps FSM state to (command string, explanation)
STATE_COMMANDS = {
    "CRUISE":      ("GO",         "Path is clear"),
    "CAUTION":     ("SLOW",       "Obstacle nearby - proceed with caution"),
    "AVOID_LEFT":  ("VEER LEFT",  "Safer to veer left"),
    "AVOID_RIGHT": ("VEER RIGHT", "Safer to veer right"),
    "STOPPED":     ("STOP",       "Obstacle dead ahead — stop"),
}


class NavigationFSM:
    """Five-state FSM with hysteresis and dwell-time gating."""

    def __init__(self):
        self.state = "CRUISE"
        self._last_change_time = 0.0

    def update(self, risks):
        """Feed new [L, C, R] risks, returns a Decision."""
        candidate = self._evaluate_candidate(risks)
        self._apply_with_dwell_gate(candidate)
        return self._build_decision(risks)

    def _evaluate_candidate(self, risks):
        """Figure out what state we should be in right now."""
        left, center, right = risks
        peak_risk = max(risks)

        # thresholds shift down if we're already in that state (hysteresis)
        stop_thresh    = self._effective_threshold("STOPPED",  STOP_THRESHOLD)
        avoid_thresh   = self._effective_threshold("AVOID",    AVOID_THRESHOLD)
        caution_thresh = self._effective_threshold("CAUTION",  CAUTION_THRESHOLD)

        # check top-down by severity
        if center >= stop_thresh:
            return "STOPPED"

        if center >= avoid_thresh:
            # steer toward whichever side has less risk
            return "AVOID_LEFT" if left < right else "AVOID_RIGHT"

        if peak_risk >= caution_thresh:
            return "CAUTION"

        return "CRUISE"

    def _effective_threshold(self, state_prefix, base_threshold):
        """Lower threshold by HYSTERESIS if we're already in this state."""
        if self.state.startswith(state_prefix):
            return base_threshold - HYSTERESIS
        return base_threshold

    def _apply_with_dwell_gate(self, candidate):
        """Only allow state change if dwell time has passed. STOP always goes through."""
        if candidate == self.state:
            return

        now = time.time()
        elapsed_ms = (now - self._last_change_time) * 1000

        if candidate == "STOPPED" or elapsed_ms >= MIN_DWELL_MS:
            self.state = candidate
            self._last_change_time = now

    def _build_decision(self, risks):
        """Turn current state into a Decision."""
        command, explanation = STATE_COMMANDS[self.state]
        urgency = max(risks)
        return Decision(command, urgency, explanation)
