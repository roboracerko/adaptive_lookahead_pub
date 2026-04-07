"""
Track loading and management utilities.
Supports loading centerline from CSV, YAML, or ROS Path message.
"""

import csv
import yaml
import os
from typing import List, Tuple, Optional
from nav_msgs.msg import Path


class Track:
    """Track representation with centerline and boundaries."""
    
    def __init__(self):
        self.centerline: List[Tuple[float, float]] = []
        self.left_boundary: List[Tuple[float, float]] = []
        self.right_boundary: List[Tuple[float, float]] = []
        self.length: float = 0.0
        self.half_width: float = 1.0  # Default track half-width
    
    def load_centerline_from_csv(self, csv_path: str,
                                  x_col: str = 'x_m',
                                  y_col: str = 'y_m') -> bool:
        """
        Load centerline from CSV file.
        
        Args:
            csv_path: Path to CSV file
            x_col: Column name for x coordinates
            y_col: Column name for y coordinates
        
        Returns:
            True if successful, False otherwise
        """
        if not os.path.exists(csv_path):
            return False
        
        try:
            with open(csv_path, 'r') as f:
                reader = csv.DictReader(f)
                self.centerline = []
                for row in reader:
                    x = float(row[x_col])
                    y = float(row[y_col])
                    self.centerline.append((x, y))
            
            # Compute path length
            self.length = self._compute_length(self.centerline)
            return True
        except Exception as e:
            print(f"Error loading centerline from CSV: {e}")
            return False
    
    def load_centerline_from_path_msg(self, path_msg: Path) -> bool:
        """
        Load centerline from nav_msgs/Path message.
        
        Args:
            path_msg: Path message containing poses
        
        Returns:
            True if successful, False otherwise
        """
        try:
            self.centerline = []
            for pose_stamped in path_msg.poses:
                x = pose_stamped.pose.position.x
                y = pose_stamped.pose.position.y
                self.centerline.append((x, y))
            
            self.length = self._compute_length(self.centerline)
            return True
        except Exception as e:
            print(f"Error loading centerline from Path: {e}")
            return False
    
    def load_from_yaml(self, yaml_path: str) -> bool:
        """
        Load track configuration from YAML file.
        
        Expected format:
        track:
          centerline_file: path/to/centerline.csv
          half_width: 1.5
          length: 200.0
        
        Args:
            yaml_path: Path to YAML file
        
        Returns:
            True if successful, False otherwise
        """
        if not os.path.exists(yaml_path):
            return False
        
        try:
            with open(yaml_path, 'r') as f:
                config = yaml.safe_load(f)
            
            track_config = config.get('track', {})
            
            # Load centerline
            centerline_file = track_config.get('centerline_file')
            if centerline_file:
                if not self.load_centerline_from_csv(centerline_file):
                    return False
            
            # Load track parameters
            self.half_width = float(track_config.get('half_width', 1.0))
            self.length = float(track_config.get('length', self.length))
            
            return True
        except Exception as e:
            print(f"Error loading track from YAML: {e}")
            return False
    
    def _compute_length(self, path: List[Tuple[float, float]]) -> float:
        """Compute total length of path."""
        if len(path) < 2:
            return 0.0
        
        total = 0.0
        for i in range(len(path) - 1):
            dx = path[i + 1][0] - path[i][0]
            dy = path[i + 1][1] - path[i][1]
            total += (dx * dx + dy * dy) ** 0.5
        
        return total
    
    def is_valid(self) -> bool:
        """Check if track has valid centerline."""
        return len(self.centerline) >= 2
