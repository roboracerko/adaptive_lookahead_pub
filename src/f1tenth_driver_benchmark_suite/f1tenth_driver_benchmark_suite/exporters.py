"""
Export utilities for metrics data.
Supports CSV and JSON export formats.
"""

import json
import csv
import os
from datetime import datetime
from typing import Dict, List, Any


class MetricsExporter:
    """Export metrics data to various formats."""
    
    def __init__(self, output_dir: str):
        """
        Initialize exporter.
        
        Args:
            output_dir: Base directory for output files
        """
        self.output_dir = output_dir
        os.makedirs(output_dir, exist_ok=True)
        
        # Create subdirectories
        self.lap_dir = os.path.join(output_dir, 'laps')
        self.run_dir = os.path.join(output_dir, 'runs')
        os.makedirs(self.lap_dir, exist_ok=True)
        os.makedirs(self.run_dir, exist_ok=True)
    
    def export_lap_summary(self, lap_num: int, summary: Dict[str, Any]) -> str:
        """
        Export lap summary to JSON file.
        
        Args:
            lap_num: Lap number
            summary: Dictionary containing lap metrics
        
        Returns:
            Path to exported file
        """
        filename = f"lap_{lap_num:03d}.json"
        filepath = os.path.join(self.lap_dir, filename)
        
        with open(filepath, 'w') as f:
            json.dump(summary, f, indent=2)
        
        return filepath
    
    def export_run_summary(self, run_id: str, summary: Dict[str, Any]) -> str:
        """
        Export run summary to JSON file.
        
        Args:
            run_id: Unique run identifier
            summary: Dictionary containing run metrics
        
        Returns:
            Path to exported file
        """
        filename = f"run_{run_id}.json"
        filepath = os.path.join(self.run_dir, filename)
        
        with open(filepath, 'w') as f:
            json.dump(summary, f, indent=2)
        
        return filepath
    
    def export_lap_csv(self, lap_num: int, summary: Dict[str, Any]) -> str:
        """
        Export lap summary to CSV file.
        
        Args:
            lap_num: Lap number
            summary: Dictionary containing lap metrics
        
        Returns:
            Path to exported file
        """
        filename = f"lap_{lap_num:03d}.csv"
        filepath = os.path.join(self.lap_dir, filename)
        
        with open(filepath, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['Metric', 'Value'])
            for key, value in summary.items():
                writer.writerow([key, value])
        
        return filepath
    
    def export_all_laps_csv(self, all_laps: List[Dict[str, Any]]) -> str:
        """
        Export all lap summaries to a single CSV file.
        
        Args:
            all_laps: List of lap summary dictionaries
        
        Returns:
            Path to exported file
        """
        if not all_laps:
            return ""
        
        filename = "all_laps_summary.csv"
        filepath = os.path.join(self.output_dir, filename)
        
        # Get all unique keys from all laps
        all_keys = set()
        for lap in all_laps:
            all_keys.update(lap.keys())
        all_keys = sorted(list(all_keys))
        
        with open(filepath, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=all_keys)
            writer.writeheader()
            for lap in all_laps:
                writer.writerow(lap)
        
        return filepath
    
    def export_trajectory_csv(self, trajectory: List[tuple]) -> str:
        """
        Export vehicle trajectory to CSV file.
        
        Args:
            trajectory: List of (x, y, time, speed, lateral_error) tuples
        
        Returns:
            Path to exported file
        """
        filename = "trajectory.csv"
        filepath = os.path.join(self.output_dir, filename)
        
        with open(filepath, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['x', 'y', 'time', 'speed', 'lateral_error'])
            for point in trajectory:
                writer.writerow(point)
        
        return filepath
    
    def export_trajectory_json(self, trajectory: List[tuple]) -> str:
        """
        Export vehicle trajectory to JSON file.
        
        Args:
            trajectory: List of (x, y, time, speed, lateral_error) tuples
        
        Returns:
            Path to exported file
        """
        filename = "trajectory.json"
        filepath = os.path.join(self.output_dir, filename)
        
        trajectory_data = [
            {
                "x": point[0],
                "y": point[1],
                "time": point[2],
                "speed": point[3],
                "lateral_error": point[4]
            }
            for point in trajectory
        ]
        
        with open(filepath, 'w') as f:
            json.dump(trajectory_data, f, indent=2)
        
        return filepath
    
    def export_timeseries_csv(self, timeseries_data: List[Dict[str, Any]]) -> str:
        """
        Export time-series data to CSV.
        
        Args:
            timeseries_data: List of dictionaries with timestamp and metrics
        
        Returns:
            Path to exported file
        """
        if not timeseries_data:
            return ""
        
        filename = f"timeseries_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        filepath = os.path.join(self.output_dir, filename)
        
        # Get all keys
        all_keys = set()
        for entry in timeseries_data:
            all_keys.update(entry.keys())
        all_keys = sorted(list(all_keys))
        
        with open(filepath, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=all_keys)
            writer.writeheader()
            for entry in timeseries_data:
                writer.writerow(entry)
        
        return filepath
