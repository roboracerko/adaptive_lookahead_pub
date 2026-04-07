"""
Lap detection and management.
Handles start/finish line crossing detection and lap state tracking.
"""

import math
from typing import Tuple, Optional, Callable
from .geometry import seg_intersect, project_point_to_segment


class LapManager:
    """
    Manages lap detection using start/finish line crossing.
    Improved to detect crossing based on lateral line (perpendicular to track direction).
    """
    
    def __init__(self,
                 start_line_p1: Tuple[float, float],
                 start_line_p2: Tuple[float, float],
                 min_lap_time_s: float = 3.0,
                 direction_check: bool = True,
                 crossing_threshold_m: float = 0.5):
        """
        Initialize lap manager.
        
        Args:
            start_line_p1: First point of start/finish line (lateral line)
            start_line_p2: Second point of start/finish line (lateral line)
            min_lap_time_s: Minimum time between laps to avoid false detections
            direction_check: If True, check that vehicle is moving forward
            crossing_threshold_m: Distance threshold for crossing detection [m]
        """
        self.start_A = start_line_p1
        self.start_B = start_line_p2
        self.min_lap_time_s = min_lap_time_s
        self.direction_check = direction_check
        self.crossing_threshold_m = crossing_threshold_m
        
        # Compute start line direction (normalized)
        dx = self.start_B[0] - self.start_A[0]
        dy = self.start_B[1] - self.start_A[1]
        line_length = math.hypot(dx, dy)
        if line_length > 1e-6:
            self.line_dir = (dx / line_length, dy / line_length)
        else:
            self.line_dir = (1.0, 0.0)
        
        # Normal vector (perpendicular to start line, pointing in forward direction)
        # Rotate line_dir by 90 degrees counter-clockwise
        self.line_normal = (-self.line_dir[1], self.line_dir[0])
        
        self.lap_count = 0
        self.last_lap_time = None
        self.last_crossing_time = None
        self.lap_start_time = None
        
        # Previous position and side for crossing detection
        self.prev_pos: Optional[Tuple[float, float]] = None
        self.prev_yaw: Optional[float] = None
        self.prev_side: Optional[float] = None  # Signed distance to start line (positive = forward side)
    
    def _compute_signed_distance_to_line(self, pos: Tuple[float, float]) -> float:
        """
        Compute signed distance from point to start line.
        
        Returns:
            Signed distance: positive if on forward side, negative if on backward side
        """
        # Vector from start_A to pos
        vx = pos[0] - self.start_A[0]
        vy = pos[1] - self.start_A[1]
        
        # Project onto normal vector (signed distance)
        signed_dist = vx * self.line_normal[0] + vy * self.line_normal[1]
        return signed_dist
    
    def _is_on_start_line(self, pos: Tuple[float, float]) -> bool:
        """
        Check if point is close to start line (within threshold).
        """
        # Project point onto start line segment
        proj, t, d2 = project_point_to_segment(pos, self.start_A, self.start_B)
        dist = math.sqrt(d2)
        
        # Check if point is within threshold distance from line
        # and projection is within segment bounds
        return dist <= self.crossing_threshold_m and 0.0 <= t <= 1.0
    
    def update(self, pos: Tuple[float, float], yaw: float, time: float) -> bool:
        """
        Update with new vehicle position and check for lap crossing.
        
        Improved logic:
        1. Detect when vehicle crosses start line (any direction)
        2. For first lap: count if crossing from backward to forward OR if starting on forward side and crossing to backward then back to forward
        3. For subsequent laps: count backward → forward crossings only (completing a full circuit)
        4. Verify minimum lap time has passed
        5. Optionally verify forward direction
        
        Args:
            pos: Current position (x, y)
            yaw: Current yaw angle (radians)
            time: Current time (seconds)
        
        Returns:
            True if new lap detected, False otherwise
        """
        crossed = False
        
        if self.prev_pos is not None:
            # Compute signed distance to start line for both positions
            prev_dist = self._compute_signed_distance_to_line(self.prev_pos)
            curr_dist = self._compute_signed_distance_to_line(pos)
            
            # Check if vehicle crossed the start line (any direction)
            sign_changed = False
            crossing_direction = None
            
            if prev_dist <= 0.0 and curr_dist > 0.0:
                # Crossing from backward to forward
                sign_changed = True
                crossing_direction = 'backward_to_forward'
            elif prev_dist > 0.0 and curr_dist <= 0.0:
                # Crossing from forward to backward
                sign_changed = True
                crossing_direction = 'forward_to_backward'
            
            if sign_changed:
                # Check if trajectory actually intersects the line segment
                if seg_intersect(self.prev_pos, pos, self.start_A, self.start_B):
                    # For lap counting, we use a simpler approach:
                    # Count ANY crossing (both directions) but only if minimum lap time has passed
                    # This handles the case where vehicle starts on either side of the line
                    
                    if self.last_crossing_time is None:
                        # First crossing - initialize timing
                        if self.lap_start_time is None:
                            self.lap_start_time = time
                        # For first lap:
                        # - If backward → forward: count as Lap 1 (vehicle completed a full circuit)
                        # - If forward → backward: record the crossing time but don't count yet
                        #   (wait for backward → forward to complete the lap)
                        if crossing_direction == 'backward_to_forward':
                            # Verify forward direction if enabled
                            if not self.direction_check or self._is_forward_direction(yaw, pos):
                                crossed = True
                                self.last_crossing_time = time
                        elif crossing_direction == 'forward_to_backward':
                            # Record the crossing time but don't count as lap yet
                            # This will allow the next backward → forward to be counted as Lap 1
                            self.last_crossing_time = time
                    else:
                        # Subsequent crossings - check minimum lap time
                        elapsed = time - self.last_crossing_time
                        if elapsed >= self.min_lap_time_s:
                            # For subsequent laps, only count backward → forward (completing a full circuit)
                            if crossing_direction == 'backward_to_forward':
                                # Verify forward direction if enabled
                                if not self.direction_check or self._is_forward_direction(yaw, pos):
                                    crossed = True
                                    self.last_crossing_time = time
                            # Don't count forward → backward as a lap (vehicle is going backwards)
        
        if crossed:
            self.lap_count += 1
            if self.lap_start_time is not None:
                self.last_lap_time = time - self.lap_start_time
            else:
                self.last_lap_time = None
            # Set lap start time for next lap
            self.lap_start_time = time
        
        self.prev_pos = pos
        self.prev_yaw = yaw
        self.prev_side = self._compute_signed_distance_to_line(pos) if pos is not None else None
        
        return crossed
    
    def _is_forward_direction(self, yaw: float, pos: Tuple[float, float]) -> bool:
        """
        Check if vehicle is moving in forward direction.
        
        Improved logic:
        - Vehicle should be moving roughly perpendicular to start line
        - Direction should be from backward side to forward side
        """
        if not self.direction_check:
            return True
        
        # Compute vehicle forward direction vector
        forward_x = math.cos(yaw)
        forward_y = math.sin(yaw)
        
        # Check if forward direction aligns with line normal (forward direction)
        # Dot product should be positive (moving in forward direction)
        dot_product = forward_x * self.line_normal[0] + forward_y * self.line_normal[1]
        
        # Vehicle should be moving forward (dot product > 0.1, roughly 84 degrees)
        # Lowered threshold to be more lenient for vehicles crossing at an angle
        return dot_product > 0.1
    
    def get_lap_count(self) -> int:
        """Get current lap count."""
        return self.lap_count
    
    def get_current_lap_time(self, current_time: float) -> Optional[float]:
        """Get elapsed time for current lap."""
        if self.lap_start_time is None:
            return None
        return current_time - self.lap_start_time
    
    def reset(self):
        """Reset lap counter and state."""
        self.lap_count = 0
        self.last_lap_time = None
        self.last_crossing_time = None
        self.lap_start_time = None
        self.prev_pos = None
        self.prev_yaw = None
        self.prev_side = None
