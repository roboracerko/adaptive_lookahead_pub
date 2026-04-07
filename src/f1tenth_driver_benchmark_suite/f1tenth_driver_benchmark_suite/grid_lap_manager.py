"""
Grid-based Lap detection and management.
Uses grid-based tracking to detect when vehicle completes a full lap.
"""

import math
from typing import Tuple, Optional, Set, List
from collections import deque


class GridLapManager:
    """
    Manages lap detection using grid-based tracking.
    
    Algorithm:
    1. Convert track centerline to grid cells
    2. Define start/finish line as grid cells
    3. Track vehicle's visited grid cells
    4. Detect when vehicle crosses start line after visiting all track cells
    """
    
    def __init__(self,
                 centerline: List[Tuple[float, float]],
                 start_line_p1: Tuple[float, float],
                 start_line_p2: Tuple[float, float],
                 grid_resolution: float = 0.5,  # Grid cell size in meters
                 start_line_width: float = 2.0,  # Start line width in meters
                 min_lap_time_s: float = 5.0,
                 coverage_threshold: float = 0.7):  # Minimum track coverage to count as lap
        """
        Initialize grid-based lap manager.
        
        Args:
            centerline: List of (x, y) points forming the track centerline
            start_line_p1: First point of start/finish line
            start_line_p2: Second point of start/finish line
            grid_resolution: Size of each grid cell in meters
            start_line_width: Width of start line area in meters
            min_lap_time_s: Minimum time between laps to avoid false detections
            coverage_threshold: Minimum fraction of track cells that must be visited (0.0-1.0)
        """
        self.centerline = centerline
        self.start_line_p1 = start_line_p1
        self.start_line_p2 = start_line_p2
        self.grid_resolution = grid_resolution
        self.start_line_width = start_line_width
        self.min_lap_time_s = min_lap_time_s
        self.coverage_threshold = coverage_threshold
        
        # Grid setup
        self._setup_grid()
        
        # State
        self.lap_count = 0
        self.last_lap_time = None
        self.lap_start_time = None
        self.last_crossing_time = None
        
        # Tracking
        self.visited_cells: Set[Tuple[int, int]] = set()
        self.current_lap_cells: Set[Tuple[int, int]] = set()
        self.was_in_start_zone = False
        self.currently_in_start_zone = False
        
        # Previous position
        self.prev_pos: Optional[Tuple[float, float]] = None
        self.prev_grid: Optional[Tuple[int, int]] = None
    
    def _setup_grid(self):
        """Setup grid system from centerline."""
        if len(self.centerline) < 2:
            raise ValueError("Centerline must have at least 2 points")
        
        # Find bounding box of track
        min_x = min(p[0] for p in self.centerline)
        max_x = max(p[0] for p in self.centerline)
        min_y = min(p[1] for p in self.centerline)
        max_y = max(p[1] for p in self.centerline)
        
        # Expand bounding box slightly
        margin = 5.0
        self.grid_min_x = min_x - margin
        self.grid_max_x = max_x + margin
        self.grid_min_y = min_y - margin
        self.grid_max_y = max_y + margin
        
        # Grid dimensions
        self.grid_width = int((self.grid_max_x - self.grid_min_x) / self.grid_resolution) + 1
        self.grid_height = int((self.grid_max_y - self.grid_min_y) / self.grid_resolution) + 1
        
        # Convert centerline to grid cells
        self.track_cells: Set[Tuple[int, int]] = set()
        for point in self.centerline:
            grid_x, grid_y = self._world_to_grid(point[0], point[1])
            # Add cell and neighbors (to account for track width)
            for dx in [-1, 0, 1]:
                for dy in [-1, 0, 1]:
                    self.track_cells.add((grid_x + dx, grid_y + dy))
        
        # Define start line zone (grid cells near start line)
        self.start_zone_cells: Set[Tuple[int, int]] = set()
        # Create start line segment and find nearby grid cells
        start_mid_x = (self.start_line_p1[0] + self.start_line_p2[0]) / 2.0
        start_mid_y = (self.start_line_p1[1] + self.start_line_p2[1]) / 2.0
        
        # Add cells within start_line_width of start line midpoint
        start_grid_x, start_grid_y = self._world_to_grid(start_mid_x, start_mid_y)
        cells_radius = int(self.start_line_width / self.grid_resolution) + 1
        
        for dx in range(-cells_radius, cells_radius + 1):
            for dy in range(-cells_radius, cells_radius + 1):
                dist = math.hypot(dx * self.grid_resolution, dy * self.grid_resolution)
                if dist <= self.start_line_width:
                    self.start_zone_cells.add((start_grid_x + dx, start_grid_y + dy))
        
        # Total track cells (for coverage calculation)
        self.total_track_cells = len(self.track_cells)
    
    def _world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """Convert world coordinates to grid coordinates."""
        grid_x = int((x - self.grid_min_x) / self.grid_resolution)
        grid_y = int((y - self.grid_min_y) / self.grid_resolution)
        return (grid_x, grid_y)
    
    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """Convert world coordinates to grid coordinates (public)."""
        return self._world_to_grid(x, y)
    
    def _is_in_start_zone(self, grid: Tuple[int, int]) -> bool:
        """Check if grid cell is in start zone."""
        return grid in self.start_zone_cells
    
    def is_in_start_zone(self, grid: Tuple[int, int]) -> bool:
        """Check if grid cell is in start zone (public)."""
        return self._is_in_start_zone(grid)
    
    def _is_on_track(self, grid: Tuple[int, int]) -> bool:
        """Check if grid cell is on track."""
        return grid in self.track_cells
    
    def update(self, pos: Tuple[float, float], time: float) -> bool:
        """
        Update with new vehicle position and check for lap completion.
        
        Algorithm:
        1. Convert position to grid coordinates
        2. Track visited cells
        3. Detect start zone entry/exit
        4. When entering start zone after visiting sufficient track cells, count as lap
        
        Args:
            pos: Current position (x, y)
            time: Current time (seconds)
        
        Returns:
            True if new lap detected, False otherwise
        """
        # Convert to grid coordinates
        grid = self._world_to_grid(pos[0], pos[1])
        
        # Track visited cells
        if self._is_on_track(grid):
            self.visited_cells.add(grid)
            self.current_lap_cells.add(grid)
        
        # Check if in start zone
        self.currently_in_start_zone = self._is_in_start_zone(grid)
        
        crossed = False
        
        # Detect start zone crossing: was outside, now inside
        if not self.was_in_start_zone and self.currently_in_start_zone:
            # Entered start zone
            # Check if we've visited enough track cells (coverage check)
            if len(self.current_lap_cells) > 0:
                coverage = len(self.current_lap_cells) / self.total_track_cells
                
                # Check minimum lap time
                elapsed = time - self.last_crossing_time if self.last_crossing_time else float('inf')
                
                if coverage >= self.coverage_threshold and elapsed >= self.min_lap_time_s:
                    # Lap completed!
                    crossed = True
                    self.last_crossing_time = time
                    # Reset for next lap
                    self.current_lap_cells.clear()
        
        # Update state
        self.was_in_start_zone = self.currently_in_start_zone
        self.prev_pos = pos
        self.prev_grid = grid
        
        if crossed:
            self.lap_count += 1
            if self.lap_start_time is not None:
                self.last_lap_time = time - self.lap_start_time
            else:
                self.last_lap_time = None
            # Set lap start time for next lap
            self.lap_start_time = time
        
        return crossed
    
    def get_lap_count(self) -> int:
        """Get current lap count."""
        return self.lap_count
    
    def get_current_lap_time(self, current_time: float) -> Optional[float]:
        """Get elapsed time for current lap."""
        if self.lap_start_time is None:
            return None
        return current_time - self.lap_start_time
    
    def get_track_coverage(self) -> float:
        """Get current track coverage (0.0-1.0)."""
        if self.total_track_cells == 0:
            return 0.0
        return len(self.current_lap_cells) / self.total_track_cells
    
    def reset(self):
        """Reset lap counter and state."""
        self.lap_count = 0
        self.last_lap_time = None
        self.last_crossing_time = None
        self.lap_start_time = None
        self.visited_cells.clear()
        self.current_lap_cells.clear()
        self.was_in_start_zone = False
        self.currently_in_start_zone = False
        self.prev_pos = None
        self.prev_grid = None
