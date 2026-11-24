"""
================================================================================
  RAMP DETECTION & SPEED GOVERNOR MODULE
================================================================================
  Detects when the vehicle is ascending, traversing, or descending a ramp /
  elevation incline based on IMU pitch and tilt angles.
  When a ramp is detected, enforces a safe cruising speed limit (e.g. 2.0 m/s)
  with hysteresis and an apex transition hold latch.
================================================================================
"""

import math
from typing import Dict, Any

try:
    from .planner_config import (
        ENABLE_RAMP_DETECTION,
        RAMP_SPEED_LIMIT_MPS,
        RAMP_ENTER_PITCH_DEG,
        RAMP_ENTER_TILT_DEG,
        RAMP_EXIT_PITCH_DEG,
        RAMP_EXIT_TILT_DEG,
        RAMP_HOLD_TIME_S,
    )
except ImportError:
    from planner_config import (
        ENABLE_RAMP_DETECTION,
        RAMP_SPEED_LIMIT_MPS,
        RAMP_ENTER_PITCH_DEG,
        RAMP_ENTER_TILT_DEG,
        RAMP_EXIT_PITCH_DEG,
        RAMP_EXIT_TILT_DEG,
        RAMP_HOLD_TIME_S,
    )


class RampDetector:
    """
    Ramp detection and speed governance based on filtered IMU attitude.
    Replaces rollover detection to prevent unwanted countersteering and power cut
    on ramps, instead capping speed to a safe traversing limit.
    """

    def __init__(
        self,
        enabled: bool = ENABLE_RAMP_DETECTION,
        speed_limit_mps: float = RAMP_SPEED_LIMIT_MPS,
        enter_pitch_deg: float = RAMP_ENTER_PITCH_DEG,
        enter_tilt_deg: float = RAMP_ENTER_TILT_DEG,
        exit_pitch_deg: float = RAMP_EXIT_PITCH_DEG,
        exit_tilt_deg: float = RAMP_EXIT_TILT_DEG,
        hold_time_s: float = RAMP_HOLD_TIME_S,
        filter_alpha: float = 0.25,
    ):
        self.enabled = enabled
        self.speed_limit_mps = speed_limit_mps
        self.enter_pitch_deg = enter_pitch_deg
        self.enter_tilt_deg = enter_tilt_deg
        self.exit_pitch_deg = exit_pitch_deg
        self.exit_tilt_deg = exit_tilt_deg
        self.hold_time_s = hold_time_s
        self.filter_alpha = filter_alpha

        # Internal state
        self.is_on_ramp: bool = False
        self.hold_timer: float = 0.0
        self.filtered_pitch: float = 0.0
        self.filtered_roll: float = 0.0
        self._initialized: bool = False

    def reset(self) -> None:
        """Resets detector state and timers."""
        self.is_on_ramp = False
        self.hold_timer = 0.0
        self.filtered_pitch = 0.0
        self.filtered_roll = 0.0
        self._initialized = False

    def update(
        self,
        pitch_rad: float,
        roll_rad: float,
        dt: float = 0.008,
    ) -> Dict[str, Any]:
        """
        Evaluates current attitude for ramp traversal.

        Args:
            pitch_rad: Vehicle pitch angle in radians (+ = nose up, - = nose down)
            roll_rad: Vehicle roll angle in radians (+ = lean right, - = lean left)
            dt: Timestep duration (seconds)

        Returns:
            Dictionary with ramp detection state, speed limit, and attitude angles.
        """
        if not self._initialized:
            self.filtered_pitch = pitch_rad
            self.filtered_roll = roll_rad
            self._initialized = True
        else:
            # Low-pass filter to reject high-frequency suspension vibration / bumps
            a = self.filter_alpha
            self.filtered_pitch = a * pitch_rad + (1.0 - a) * self.filtered_pitch
            self.filtered_roll = a * roll_rad + (1.0 - a) * self.filtered_roll

        pitch_deg = abs(math.degrees(self.filtered_pitch))
        roll_deg = abs(math.degrees(self.filtered_roll))
        tilt_deg = math.degrees(math.hypot(self.filtered_pitch, self.filtered_roll))

        if not self.enabled:
            return {
                "is_on_ramp": False,
                "speed_limit": self.speed_limit_mps,
                "pitch_deg": round(pitch_deg, 2),
                "roll_deg": round(roll_deg, 2),
                "tilt_deg": round(tilt_deg, 2),
                "hold_timer": 0.0,
            }

        # Check entry trigger: pitch or tilt exceeds entry threshold
        is_incline = (pitch_deg >= self.enter_pitch_deg) or (tilt_deg >= self.enter_tilt_deg)

        if is_incline:
            self.is_on_ramp = True
            self.hold_timer = self.hold_time_s
        elif self.hold_timer > 0.0:
            # Latch hold: maintain ramp mode while transitioning across flat apex or leaving ramp
            self.hold_timer = max(0.0, self.hold_timer - dt)
            if self.hold_timer <= 0.0:
                # Check exit hysteresis
                if (pitch_deg < self.exit_pitch_deg) and (tilt_deg < self.exit_tilt_deg):
                    self.is_on_ramp = False
                else:
                    # Still above exit hysteresis floor, re-arm hold timer
                    self.hold_timer = self.hold_time_s
        else:
            # Check exit threshold
            if (pitch_deg < self.exit_pitch_deg) and (tilt_deg < self.exit_tilt_deg):
                self.is_on_ramp = False

        return {
            "is_on_ramp": self.is_on_ramp,
            "speed_limit": self.speed_limit_mps,
            "pitch_deg": round(pitch_deg, 2),
            "roll_deg": round(roll_deg, 2),
            "tilt_deg": round(tilt_deg, 2),
            "hold_timer": round(self.hold_timer, 3),
        }


# Backwards compatibility alias
AntiRolloverController = RampDetector
