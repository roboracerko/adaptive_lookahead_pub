"""
Progress-based Lap detection and management.
Uses centerline projection to compute arc length coordinate (s) and detects
lap completion via wrap-around (s: L-ε -> ε).
"""

import math
from dataclasses import dataclass
from enum import Enum, auto
from typing import List, Tuple, Optional

from .geometry import (
    compute_polyline_prefix_lengths,
    project_point_to_polyline,
    circular_progress,
)

Point2 = Tuple[float, float]


class LapState(Enum):
    """State machine for lap detection."""
    INIT = auto()          # Initial: wait until leaving start zone
    ARMED = auto()         # Outside start zone: ready for lap detection
    IN_START_ZONE = auto() # Inside start zone: hysteresis state
    COOLDOWN = auto()      # After lap: prevent immediate re-detection


@dataclass
class ProgressLapConfig:
    """Configuration for progress-based lap detection."""
    # Minimum lap time
    min_lap_time_s: float = 5.0

    # Start zone in s-space (around start_s)
    start_s: float = 0.0
    start_zone_s: float = 5.0  # ±start_zone_s around start_s

    # Wrap-around detection margin
    wrap_margin_s: float = 3.0  # prev_s >= L-ε and curr_s <= ε

    # Motion filter (ignore noise/standstill)
    min_progress_s: float = 0.10  # Minimum forward progress per tick

    # Teleport/reset detection (optional)
    teleport_distance_m: float = 3.0
    teleport_progress_s: float = 10.0

    # Cooldown after lap
    cooldown_s: float = 1.0  # Ignore start zone jitter after lap


class ProgressLapManager:
    """
    Progress-based lap detection using centerline projection.
    
    Algorithm:
    1. Project vehicle position to centerline -> s (arc length, 0..L)
    2. Detect wrap-around: prev_s >= L-ε and curr_s <= ε
    3. State machine prevents false positives from:
       - Initial spawn in start zone
       - Jitter near start line
       - Teleport/reset events
    """
    
    def __init__(
        self,
        centerline: List[Point2],
        config: Optional[ProgressLapConfig] = None,
        logger=None,
    ):
        """
        Initialize progress-based lap manager.
        
        Args:
            centerline: List of (x, y) points forming track centerline
            config: Configuration (uses defaults if None)
        """
        if len(centerline) < 2:
            raise ValueError("centerline must have at least 2 points")
        
        self.centerline = centerline
        self.prefix = compute_polyline_prefix_lengths(centerline)
        self.length = self.prefix[-1]
        if self.length <= 1e-6:
            raise ValueError("centerline total length too small")
        
        self.cfg = config or ProgressLapConfig()
        self.logger = logger  # Optional logger for debug output
        
        # State
        self.state = LapState.INIT
        self.lap_count = 0
        self.last_lap_time: Optional[float] = None
        self.lap_start_time: Optional[float] = None
        self.last_lap_crossing_time: Optional[float] = None
        self.cooldown_until: Optional[float] = None
        
        # Previous state
        self.prev_pos: Optional[Point2] = None
        self.prev_s: Optional[float] = None
        self.prev_time: Optional[float] = None
    
    def _is_in_start_zone_s(self, s: float) -> bool:
        """Check if s is in start zone (circular distance)."""
        dz = self.cfg.start_zone_s
        start = (self.cfg.start_s % self.length)
        
        # Circular distance
        ds = abs(s - start)
        ds = min(ds, self.length - ds)
        return ds <= dz
    
    def _wrap_crossed(self, prev_s: float, curr_s: float, curr_s_raw: Optional[float] = None) -> bool:
        """
        Check if wrap-around occurred (end -> start).
        
        Args:
            prev_s: Previous normalized s value [0, length) (but may exceed due to numerical issues)
            curr_s: Current normalized s value [0, length)
            curr_s_raw: Current raw s value before normalization (optional)
        """
        eps = self.cfg.wrap_margin_s
        
        # Normalize prev_s to ensure it's in [0, length) range
        # This handles cases where prev_s might equal or exceed length
        prev_s_norm = prev_s % self.length if self.length > 0 else prev_s
        
        # Standard check: prev_s near end and curr_s near start
        if (prev_s_norm >= self.length - eps) and (curr_s <= eps):
            if self.logger:
                self.logger.info(f"[DEBUG] _wrap_crossed (standard): prev_s={prev_s:.2f} (norm={prev_s_norm:.2f}), curr_s={curr_s:.2f}, length={self.length:.2f}, eps={eps:.2f} -> True")
            return True
        
        # Additional check: if curr_s_raw is provided and exceeds length,
        # it indicates a wrap-around even if normalized curr_s is not near start
        if curr_s_raw is not None and curr_s_raw >= self.length:
            # Raw s exceeded length, check if prev_s was near end
            if prev_s_norm >= self.length - eps:
                if self.logger:
                    self.logger.info(f"[DEBUG] _wrap_crossed (raw s check): prev_s={prev_s:.2f} (norm={prev_s_norm:.2f}), curr_s_raw={curr_s_raw:.2f}, length={self.length:.2f}, eps={eps:.2f} -> True")
                return True
        
        return False
    
    def _should_reset_on_jump(self, pos: Point2, s: float) -> bool:
        """Detect teleport/reset: large position or s jump."""
        if self.prev_pos is None or self.prev_s is None:
            return False
        
        # Position jump
        dx = pos[0] - self.prev_pos[0]
        dy = pos[1] - self.prev_pos[1]
        if math.hypot(dx, dy) > self.cfg.teleport_distance_m:
            return True
        
        # s jump (circular distance)
        # IMPORTANT: Exclude wrap-around from teleport detection
        # If this looks like a wrap-around (prev_s near end, s near start),
        # it's a normal lap completion, not a teleport
        if self._wrap_crossed(self.prev_s, s):
            # This is a wrap-around, not a teleport
            return False
        
        ds = abs(s - self.prev_s)
        ds = min(ds, self.length - ds)
        if ds > self.cfg.teleport_progress_s:
            return True
        
        return False
    
    def _lap_time_ok(self, time_s: float) -> bool:
        """Check if minimum lap time has passed."""
        if self.last_lap_crossing_time is None:
            result = True
        else:
            elapsed = time_s - self.last_lap_crossing_time
            result = elapsed >= self.cfg.min_lap_time_s
            if not result and self.logger:
                self.logger.info(f"[DEBUG] _lap_time_ok: False - elapsed={elapsed:.2f}s < min_lap_time_s={self.cfg.min_lap_time_s:.2f}s")
        return result
    
    def _on_lap(self, time_s: float):
        """Handle lap completion."""
        if self.logger:
            self.logger.info(f"[DEBUG] _on_lap called: before lap_count={self.lap_count}, time_s={time_s:.2f}")
        self.lap_count += 1
        if self.logger:
            self.logger.info(f"[DEBUG] _on_lap: after increment lap_count={self.lap_count}")
        self.last_lap_crossing_time = time_s
        
        if self.lap_start_time is not None:
            self.last_lap_time = time_s - self.lap_start_time
        else:
            self.last_lap_time = None
        
        self.lap_start_time = time_s
        self.cooldown_until = time_s + self.cfg.cooldown_s
        self.state = LapState.COOLDOWN
        if self.logger:
            self.logger.info(f"[DEBUG] _on_lap: state={self.state}, cooldown_until={self.cooldown_until:.2f}")
    
    def _soft_reset_state(self, time_s: float):
        """Reset state on teleport (keep lap_count)."""
        self.state = LapState.INIT
        self.cooldown_until = time_s + self.cfg.cooldown_s
        # lap_start_time is kept to continue tracking
    
    def _update_prev(self, pos: Point2, s: float, time_s: float):
        """Update previous state."""
        self.prev_pos = pos
        # Normalize prev_s to [0, length) to ensure consistent circular_progress calculation
        # This prevents issues when prev_s equals or exceeds length
        self.prev_s = s % self.length if self.length > 0 else s
        self.prev_time = time_s
    
    def update(self, pos: Point2, time_s: float) -> bool:
        """
        Update with new position and check for lap completion.
        
        Args:
            pos: Current position (x, y) in SAME FRAME as centerline
            time_s: Current time in seconds
        
        Returns:
            True if lap detected, False otherwise
        """
        # Project to centerline -> s
        proj = project_point_to_polyline(
            pos, self.centerline, prefix=self.prefix, compute_signed=False
        )
        s_raw = proj.s  # 0..length (but may exceed due to numerical issues)
        # Normalize s to [0, length) range to handle edge cases
        s = s_raw % self.length
        
        # Teleport/reset guard
        if self._should_reset_on_jump(pos, s):
            self._soft_reset_state(time_s)
            self._update_prev(pos, s, time_s)
            return False
        
        # Cooldown gate
        if self.cooldown_until is not None and time_s < self.cooldown_until:
            self._update_prev(pos, s, time_s)
            return False
        
        # Initialize lap_start_time
        if self.lap_start_time is None:
            self.lap_start_time = time_s
        
        in_start_zone = self._is_in_start_zone_s(s)
        
        # Prime prev_s (first update)
        if self.prev_s is None:
            self.state = LapState.IN_START_ZONE if in_start_zone else LapState.INIT
            self._update_prev(pos, s, time_s)
            return False
        
        # Compute forward progress (circular)
        fwd = circular_progress(self.prev_s, s, self.length)
        
        # Check for wrap-around BEFORE fwd check
        # This is critical: wrap-around should be detected even if fwd is small
        # Normalize prev_s to handle cases where it equals or exceeds length
        prev_s_norm = self.prev_s % self.length if self.length > 0 else self.prev_s
        is_wrap_candidate = (prev_s_norm >= self.length - self.cfg.wrap_margin_s) and (s <= self.cfg.wrap_margin_s)
        
        # Debug: log wrap-around candidates
        if is_wrap_candidate:
            if self.logger:
                self.logger.info(f"[DEBUG] Wrap candidate: prev_s={self.prev_s:.2f}, s={s:.2f}, s_raw={s_raw:.2f}, fwd={fwd:.2f}, state={self.state.name}")
        
        # Too small motion -> ignore lap logic
        # BUT: Allow wrap-around detection even if fwd is small (wrap-around can have small fwd due to normalization)
        if fwd < self.cfg.min_progress_s and not is_wrap_candidate:
            if self.logger and (self.prev_s >= self.length - self.cfg.wrap_margin_s or s <= self.cfg.wrap_margin_s):
                self.logger.info(f"[DEBUG] Wrap candidate rejected: fwd={fwd:.2f} < min_progress_s={self.cfg.min_progress_s:.2f}")
            self._update_prev(pos, s, time_s)
            return False
        
        crossed = False
        
        # State machine
        if self.state == LapState.INIT:
            # Initial: must leave start zone first
            if not in_start_zone:
                self.state = LapState.ARMED
        
        elif self.state == LapState.ARMED:
            # Hysteresis: entering start zone
            if in_start_zone:
                self.state = LapState.IN_START_ZONE
            
            # Core: wrap-around detection
            # Pass raw s value to handle cases where s exceeds length
            wrap_detected = self._wrap_crossed(self.prev_s, s, s_raw)
            time_ok = self._lap_time_ok(time_s)
            if wrap_detected and time_ok:
                crossed = True
            # Debug logging
            if wrap_detected and self.logger:
                self.logger.info(f"[DEBUG] ARMED: wrap_detected={wrap_detected}, time_ok={time_ok}, prev_s={self.prev_s:.2f}, s={s:.2f}, s_raw={s_raw:.2f}, crossed={crossed}")
        
        elif self.state == LapState.IN_START_ZONE:
            # Leave start zone -> ARMED
            if not in_start_zone:
                self.state = LapState.ARMED
            
            # Wrap detection in start zone: allow detection when wrap occurs
            # This is necessary because wrap-around often happens while in start zone
            # Pass raw s value to handle cases where s exceeds length
            wrap_detected = self._wrap_crossed(self.prev_s, s, s_raw)
            time_ok = self._lap_time_ok(time_s)
            if wrap_detected and time_ok:
                crossed = True
            # Debug logging
            if wrap_detected and self.logger:
                self.logger.info(f"[DEBUG] IN_START_ZONE: wrap_detected={wrap_detected}, time_ok={time_ok}, prev_s={self.prev_s:.2f}, s={s:.2f}, s_raw={s_raw:.2f}, crossed={crossed}")
        
        elif self.state == LapState.COOLDOWN:
            # Cooldown handled by timer. Exit start zone -> ARMED
            if not in_start_zone:
                self.state = LapState.ARMED
        
        if crossed:
            if self.logger:
                self.logger.info(f"[DEBUG] LAP DETECTED! Calling _on_lap(time_s={time_s:.2f}), current lap_count={self.lap_count}")
            self._on_lap(time_s)
            if self.logger:
                self.logger.info(f"[DEBUG] After _on_lap: lap_count={self.lap_count}")
        
        self._update_prev(pos, s, time_s)
        return crossed
    
    def get_lap_count(self) -> int:
        """Get current lap count."""
        return self.lap_count
    
    def get_current_lap_time(self, current_time_s: float) -> Optional[float]:
        """Get elapsed time for current lap."""
        if self.lap_start_time is None:
            return None
        return current_time_s - self.lap_start_time
    
    def reset(self):
        """Reset lap counter and state."""
        self.state = LapState.INIT
        self.lap_count = 0
        self.last_lap_time = None
        self.lap_start_time = None
        self.last_lap_crossing_time = None
        self.cooldown_until = None
        self.prev_pos = None
        self.prev_s = None
        self.prev_time = None
