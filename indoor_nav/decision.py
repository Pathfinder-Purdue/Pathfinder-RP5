"""Decision engine: fused sector risks + ToF ground check + LiDAR flank check.

Pipeline:
  1. Score three candidate directions (LEFT, FORWARD, RIGHT) from fused [L,C,R] risks.
  2. Sort safest-first and iterate candidates.
  3. For each candidate, validate:
     a. ToF ground-level clearance in that direction.
     b. LiDAR flank clearance (FL for LEFT, FR for RIGHT).
  4. First candidate that passes both checks becomes the command.
  5. If no direction passes -> STOP.
"""

import time
from dataclasses import dataclass

from indoor_nav.config import (
    STOP_THRESHOLD,
    CAUTION_THRESHOLD,
    HYSTERESIS,
    MIN_DWELL_MS,
    TOF_VETO_RISK,
    LIDAR_FLANK_SAFE,
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
    """Directional FSM with ToF ground and LiDAR flank validation."""

    def __init__(self):
        self.state = "CRUISE"
        self._last_change_time = 0.0

    def update(self, risks, lidar_flanks=None, tof_risks=None):
        """Feed [L,C,R] fused risks, optional [FL,FR] flanks, optional [L,R] ToF ground.

        Returns a Decision.
        """
        candidate = self._pick_direction(risks, lidar_flanks, tof_risks)
        self._apply_with_dwell_gate(candidate)
        return self._build_decision(risks)

    # -- direction selection --

    def _pick_direction(self, risks, lidar_flanks, tof_risks):
        """Score LEFT / FORWARD / RIGHT, validate, return best FSM state."""
        left, center, right = risks

        # candidate directions sorted safest-first
        candidates = [
            ("AVOID_LEFT",  left),
            ("CRUISE",      center),
            ("AVOID_RIGHT", right),
        ]
        candidates.sort(key=lambda x: x[1])

        for state, risk in candidates:
            # too dangerous in this direction
            if risk >= self._eff_threshold("STOPPED", STOP_THRESHOLD):
                continue

            # ToF ground validation
            if not self._tof_clear(state, tof_risks):
                continue

            # LiDAR flank validation
            if not self._flank_clear(state, lidar_flanks):
                continue

            # direction is viable
            if state == "CRUISE":
                if risk >= self._eff_threshold("CAUTION", CAUTION_THRESHOLD):
                    return "CAUTION"
                return "CRUISE"
            return state

        return "STOPPED"

    # -- validation helpers --

    def _tof_clear(self, direction, tof_risks):
        """Check ToF ground risk is below veto threshold in chosen direction."""
        if tof_risks is None:
            return True
        if direction == "AVOID_LEFT":
            return tof_risks[0] < TOF_VETO_RISK
        if direction == "AVOID_RIGHT":
            return tof_risks[1] < TOF_VETO_RISK
        # forward: both sides must be clear
        return tof_risks[0] < TOF_VETO_RISK and tof_risks[1] < TOF_VETO_RISK

    def _flank_clear(self, direction, lidar_flanks):
        """Check LiDAR flank is safe before veering that way."""
        if lidar_flanks is None:
            return True
        if direction == "AVOID_LEFT":
            return lidar_flanks[0] < LIDAR_FLANK_SAFE
        if direction == "AVOID_RIGHT":
            return lidar_flanks[1] < LIDAR_FLANK_SAFE
        return True  # forward doesn't need flank check

    # -- thresholds / state machine --

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
