"""Phase-continuous gait switching.

49x_Hexapod_test's planner reset its phase accumulator to 0 on every gait
change (walking.py's create_gait() swap), which snaps every foot straight
to the new gait's phase offset instead of walking there -- a visible foot
jump, and the "gait switch mid-stride" failure mode called out in the
project plan's section 2 (known issues) and section 6 (design). This
module fixes that: the phase accumulator never resets, and a gait change
blends each leg's phase offset and the gait's duty factor from the old
gait to the new one over exactly one full cycle, using the same quintic
smoothstep already used for foot trajectories elsewhere in this package.

This intentionally does not try to guarantee "at least 3 legs in stance
at every instant of the blend" (a stronger, phase-locked-transition
guarantee) -- that is a documented follow-up, not something a linear
per-leg offset blend can promise for arbitrary gait pairs. What it does
guarantee: the phase used to sample every leg's trajectory is continuous
(no reset, no jump) across a gait change, and re-requesting the same gait
mid-blend has no effect.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from .gait_base import GaitBase
from .ripple_gait import RippleGait
from .trajectory import smoothstep5
from .tripod_gait import TripodGait
from .wave_gait import WaveGait

GAITS: dict[str, type[GaitBase]] = {
    "tripod": TripodGait,
    "ripple": RippleGait,
    "wave": WaveGait,
}


def create_gait(name: str) -> GaitBase:
    normalized = (name or "tripod").strip().lower()
    if normalized not in GAITS:
        raise ValueError(f"unsupported gait '{name}'")
    return GAITS[normalized]()


@dataclass(frozen=True)
class BlendedGaitSample:
    name: str
    """Name of the gait this sample is blending towards (or settled on)."""
    offsets: dict[str, float]
    duty_factor: float
    blending: bool


class GaitTransitionManager:
    """Tracks a monotonic phase accumulator and blends across gait changes.

    `phase` never wraps or resets: callers that need a [0, 1) sample phase
    for a trajectory function take `phase % 1.0` themselves (see
    planner.py), same as before -- only the *offset/duty* blending is new.
    """

    def __init__(self, gait_name: str = "tripod", blend_cycles: float = 1.0) -> None:
        if blend_cycles <= 0:
            raise ValueError("blend_cycles must be positive")
        self.blend_cycles = blend_cycles
        self._current = create_gait(gait_name)
        self._target: GaitBase | None = None
        self._blend_start_phase: float = 0.0
        self.phase: float = 0.0

    @property
    def current_name(self) -> str:
        return type(self._current).__name__.replace("Gait", "").lower()

    @property
    def blending(self) -> bool:
        return self._target is not None

    @property
    def target_name(self) -> str | None:
        if self._target is None:
            return None
        return type(self._target).__name__.replace("Gait", "").lower()

    def blend_weight(self) -> float:
        """Smoothstep weight of the target gait right now (0 when not blending).

        Used by the planner to blend per-gait cycle times with the same
        profile as offsets/duty factor, so the phase rate changes smoothly
        across a switch (e.g. Tripod 0.95 s -> Wave 1.9 s cycle).
        """
        if self._target is None:
            return 0.0
        progress = (self.phase - self._blend_start_phase) / self.blend_cycles
        return smoothstep5(max(0.0, min(1.0, progress)))

    def request_gait(self, name: str) -> bool:
        """Start (or retarget) a blend towards `name`. No-op if already there."""
        requested = create_gait(name)
        if type(requested) is type(self._current) and self._target is None:
            return False
        if self._target is not None and type(requested) is type(self._target):
            return False
        self._target = requested
        self._blend_start_phase = self.phase
        return True

    def complete(self) -> bool:
        """Finish a pending blend immediately (no-op when not blending).

        Only safe while the robot stands still: then every foot is on the
        ground at its nominal point (stride 0, lift 0 - see planner.update),
        so the phase offsets and duty factor do not move any foot. The
        planner calls it in exactly that case; without it a switch requested
        at rest would stay pending until walking resumed, because the phase
        (and therefore the blend progress) is frozen while stopped.
        """
        if self._target is None:
            return False
        self._current = self._target
        self._target = None
        return True

    def advance(self, dt_phase: float) -> BlendedGaitSample:
        """Advance the phase accumulator by `dt_phase` (dt / cycle_time) and
        return the offsets/duty factor to sample trajectories with right now.
        """
        if not math.isfinite(dt_phase):
            raise ValueError("dt_phase must be finite")
        self.phase += dt_phase

        if self._target is None:
            return BlendedGaitSample(
                self.current_name, dict(self._current.offsets), self._current.duty_factor, False
            )

        progress = (self.phase - self._blend_start_phase) / self.blend_cycles
        if progress >= 1.0:
            self._current = self._target
            self._target = None
            return BlendedGaitSample(
                self.current_name, dict(self._current.offsets), self._current.duty_factor, False
            )

        smooth = smoothstep5(max(0.0, min(1.0, progress)))
        blended_offsets = {
            leg: self._current.offsets[leg]
            + smooth * (self._target.offsets[leg] - self._current.offsets[leg])
            for leg in self._current.offsets
        }
        blended_duty = self._current.duty_factor + smooth * (
            self._target.duty_factor - self._current.duty_factor
        )
        target_name = type(self._target).__name__.replace("Gait", "").lower()
        return BlendedGaitSample(target_name, blended_offsets, blended_duty, True)
