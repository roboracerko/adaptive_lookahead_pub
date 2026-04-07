#!/usr/bin/env python3
"""
Real-time visualization node for F1TENTH driver evaluation.
Uses matplotlib to display vehicle trajectory, speed, and metrics.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, String
import matplotlib
# Set backend before importing pyplot - try multiple backends
import os
# Check if DISPLAY is available
if 'DISPLAY' in os.environ:
    # Try Qt5Agg first (more reliable), then TkAgg
    try:
        matplotlib.use('Qt5Agg')
    except ImportError:
        try:
            matplotlib.use('TkAgg')
        except ImportError:
            matplotlib.use('Agg')  # Fallback to non-interactive
else:
    matplotlib.use('Agg')  # No display available

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Rectangle, Circle
import numpy as np
import json
import threading
import queue
from collections import deque
from typing import Optional, List, Tuple

# Enable interactive mode
plt.ion()


class RealtimeVisualizer(Node):
    """Real-time visualization node using matplotlib."""
    
    def __init__(self):
        super().__init__('realtime_visualizer')
        
        # Parameters
        self.odom_topic = self.declare_parameter('odom_topic', '/ego_racecar/odom').value
        self.lateral_error_topic = self.declare_parameter('lateral_error_topic', '/metrics/debug/lateral_error').value
        self.lap_summary_topic = self.declare_parameter('lap_summary_topic', '/metrics/lap_summary').value
        self.centerline_file = self.declare_parameter('centerline_file', '').value
        self.max_history = int(self.declare_parameter('max_history', 1000).value)
        self.update_rate = float(self.declare_parameter('update_rate', 2.0).value)  # Hz (reduced to avoid blocking other GUIs)
        
        # Data buffers
        self.trajectory_x = deque(maxlen=self.max_history)
        self.trajectory_y = deque(maxlen=self.max_history)
        self.speed_history = deque(maxlen=self.max_history)
        self.lateral_error_history = deque(maxlen=self.max_history)
        self.time_history = deque(maxlen=self.max_history)
        
        # Current state
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_speed = 0.0
        self.current_lateral_error = 0.0
        self.current_lap = 0
        self.current_lap_time = 0.0
        
        # Centerline data
        self.centerline_x = []
        self.centerline_y = []
        self._load_centerline()
        
        # Subscriptions
        self.create_subscription(Odometry, self.odom_topic, self.on_odom, 10)
        self.create_subscription(Float32, self.lateral_error_topic, self.on_lateral_error, 10)
        self.create_subscription(String, self.lap_summary_topic, self.on_lap_summary, 10)
        
        # Setup matplotlib
        self.setup_plots()
        
        # Show the figure window
        plt.show(block=False)
        plt.pause(0.1)  # Small pause to ensure window is created
        
        # Use ROS 2 timer instead of matplotlib animation to avoid blocking
        # This allows ROS 2 callbacks to work properly
        # Reduced update rate to avoid blocking other GUI processes
        self.update_timer = self.create_timer(
            1.0 / self.update_rate,  # Update period in seconds
            self.update_plots_timer_callback
        )
        
        # Flag to track if update is needed (avoid redundant updates)
        self.update_needed = False
        
        # Force figure to be displayed (only once at startup)
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        
        self.get_logger().info("Real-time visualizer initialized")
        self.get_logger().info(f"  Backend: {plt.get_backend()}")
        self.get_logger().info(f"  Update rate: {self.update_rate} Hz")
        self.get_logger().info(f"  Max history: {self.max_history} points")
        self.get_logger().info(f"  DISPLAY: {os.environ.get('DISPLAY', 'Not set')}")
        self.get_logger().info(f"  Using ROS 2 timer for updates (non-blocking)")
    
    def _load_centerline(self):
        """Load centerline from file if provided."""
        if not self.centerline_file:
            return
        
        try:
            import csv
            with open(self.centerline_file, 'r') as f:
                reader = csv.reader(f)
                next(reader)  # Skip header if present
                for row in reader:
                    if len(row) >= 2:
                        try:
                            x = float(row[0])
                            y = float(row[1])
                            self.centerline_x.append(x)
                            self.centerline_y.append(y)
                        except ValueError:
                            continue
            self.get_logger().info(f"Loaded {len(self.centerline_x)} centerline points")
        except Exception as e:
            self.get_logger().warn(f"Failed to load centerline: {e}")
    
    def setup_plots(self):
        """Setup matplotlib figure and subplots."""
        self.fig = plt.figure(figsize=(16, 10))
        self.fig.suptitle('F1TENTH Driver Real-time Evaluation', fontsize=16)
        
        # Top-left: Trajectory plot
        self.ax_traj = self.fig.add_subplot(2, 3, 1)
        self.ax_traj.set_title('Vehicle Trajectory')
        self.ax_traj.set_xlabel('X (m)')
        self.ax_traj.set_ylabel('Y (m)')
        self.ax_traj.grid(True, alpha=0.3)
        self.ax_traj.set_aspect('equal', adjustable='box')
        
        # Top-center: Speed over time
        self.ax_speed = self.fig.add_subplot(2, 3, 2)
        self.ax_speed.set_title('Speed')
        self.ax_speed.set_xlabel('Time (s)')
        self.ax_speed.set_ylabel('Speed (m/s)')
        self.ax_speed.grid(True, alpha=0.3)
        
        # Top-right: Lateral error over time
        self.ax_error = self.fig.add_subplot(2, 3, 3)
        self.ax_error.set_title('Lateral Error')
        self.ax_error.set_xlabel('Time (s)')
        self.ax_error.set_ylabel('Lateral Error (m)')
        self.ax_error.grid(True, alpha=0.3)
        self.ax_error.axhline(y=0, color='k', linestyle='--', alpha=0.3)
        
        # Bottom-left: Current metrics (text)
        self.ax_metrics = self.fig.add_subplot(2, 3, 4)
        self.ax_metrics.axis('off')
        self.ax_metrics.set_title('Current Metrics', fontsize=12, fontweight='bold')
        
        # Bottom-center: Speed histogram
        self.ax_hist = self.fig.add_subplot(2, 3, 5)
        self.ax_hist.set_title('Speed Distribution')
        self.ax_hist.set_xlabel('Speed (m/s)')
        self.ax_hist.set_ylabel('Frequency')
        self.ax_hist.grid(True, alpha=0.3)
        
        # Bottom-right: Lateral error histogram
        self.ax_error_hist = self.fig.add_subplot(2, 3, 6)
        self.ax_error_hist.set_title('Lateral Error Distribution')
        self.ax_error_hist.set_xlabel('Lateral Error (m)')
        self.ax_error_hist.set_ylabel('Frequency')
        self.ax_error_hist.grid(True, alpha=0.3)
        self.ax_error_hist.axvline(x=0, color='k', linestyle='--', alpha=0.3)
        
        plt.tight_layout()
        # Force redraw
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
    
    def on_odom(self, msg: Odometry):
        """Callback for odometry updates."""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        speed = np.hypot(vx, vy)
        
        stamp = msg.header.stamp
        t = stamp.sec + stamp.nanosec * 1e-9
        
        self.current_x = x
        self.current_y = y
        self.current_speed = speed
        
        self.trajectory_x.append(x)
        self.trajectory_y.append(y)
        self.speed_history.append(speed)
        self.time_history.append(t)
        
        # Mark that update is needed (don't update immediately to avoid blocking)
        # Updates will be handled by the timer callback at a controlled rate
        self.update_needed = True
    
    def on_lateral_error(self, msg: Float32):
        """Callback for lateral error updates."""
        self.current_lateral_error = msg.data
        self.lateral_error_history.append(msg.data)
        
        # Mark that update is needed (don't update immediately to avoid blocking)
        self.update_needed = True
    
    def on_lap_summary(self, msg: String):
        """Callback for lap summary updates."""
        try:
            summary = json.loads(msg.data)
            self.current_lap = summary.get('lap', 0)
            self.current_lap_time = summary.get('lap_time_s', 0.0)
            self.get_logger().info(f"Lap {self.current_lap} completed: {self.current_lap_time:.2f}s")
        except Exception as e:
            self.get_logger().warn(f"Failed to parse lap summary: {e}")
    
    def update_plots_timer_callback(self):
        """ROS 2 timer callback to update plots."""
        # Only update if new data is available (avoid redundant updates)
        if not self.update_needed:
            return
        
        try:
            self.update_plots(None)
            self.update_needed = False  # Reset flag after update
        except Exception as e:
            self.get_logger().warn(f"Error updating plots: {e}")
    
    def update_plots(self, frame):
        """Update all plots."""
        # Clear axes
        self.ax_traj.clear()
        self.ax_speed.clear()
        self.ax_error.clear()
        self.ax_metrics.clear()
        self.ax_hist.clear()
        self.ax_error_hist.clear()
        
        # Trajectory plot
        self.ax_traj.set_title('Vehicle Trajectory')
        self.ax_traj.set_xlabel('X (m)')
        self.ax_traj.set_ylabel('Y (m)')
        self.ax_traj.grid(True, alpha=0.3)
        self.ax_traj.set_aspect('equal', adjustable='box')
        
        # Plot centerline if available
        if len(self.centerline_x) > 0:
            self.ax_traj.plot(self.centerline_x, self.centerline_y, 'k--', alpha=0.3, linewidth=1, label='Centerline')
        
        # Plot trajectory
        if len(self.trajectory_x) > 0:
            traj_x = list(self.trajectory_x)
            traj_y = list(self.trajectory_y)
            # Ensure x and y have the same length (handle race conditions)
            min_len = min(len(traj_x), len(traj_y))
            if min_len > 0:
                traj_x = traj_x[:min_len]
                traj_y = traj_y[:min_len]
                self.ax_traj.plot(traj_x, traj_y, 'b-', alpha=0.6, linewidth=1, label='Trajectory')
            # Current position
            self.ax_traj.plot(self.current_x, self.current_y, 'ro', markersize=8, label='Current')
        
        self.ax_traj.legend(loc='upper right', fontsize=8)
        
        # Speed plot
        if len(self.time_history) > 0 and len(self.speed_history) > 0:
            times = list(self.time_history)
            speeds = list(self.speed_history)
            # Ensure times and speeds have the same length (handle race conditions)
            min_len = min(len(times), len(speeds))
            if min_len > 0:
                times = np.array(times[:min_len])
                speeds = np.array(speeds[:min_len])
                times = times - times[0]  # Relative time
                self.ax_speed.plot(times, speeds, 'g-', linewidth=1)
                self.ax_speed.axhline(y=np.mean(speeds), color='r', linestyle='--', alpha=0.5, label=f'Avg: {np.mean(speeds):.2f} m/s')
        
        self.ax_speed.set_title('Speed')
        self.ax_speed.set_xlabel('Time (s)')
        self.ax_speed.set_ylabel('Speed (m/s)')
        self.ax_speed.grid(True, alpha=0.3)
        self.ax_speed.legend(loc='upper right', fontsize=8)
        
        # Lateral error plot
        if len(self.time_history) > 0 and len(self.lateral_error_history) > 0:
            times = list(self.time_history)
            errors = list(self.lateral_error_history)
            # Ensure times and errors have the same length (handle race conditions)
            min_len = min(len(times), len(errors))
            if min_len > 0:
                times = np.array(times[:min_len])
                errors = np.array(errors[:min_len])
                times = times - times[0]  # Relative time
                self.ax_error.plot(times, errors, 'r-', linewidth=1)
                self.ax_error.axhline(y=0, color='k', linestyle='--', alpha=0.3)
        
        self.ax_error.set_title('Lateral Error')
        self.ax_error.set_xlabel('Time (s)')
        self.ax_error.set_ylabel('Lateral Error (m)')
        self.ax_error.grid(True, alpha=0.3)
        
        # Metrics text
        self.ax_metrics.axis('off')
        metrics_text = f"""Current Metrics:
        
Lap: {self.current_lap}
Lap Time: {self.current_lap_time:.2f} s
Speed: {self.current_speed:.2f} m/s
Lateral Error: {self.current_lateral_error:.3f} m

Trajectory Points: {len(self.trajectory_x)}
"""
        self.ax_metrics.text(0.1, 0.5, metrics_text, fontsize=10, verticalalignment='center',
                            family='monospace', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
        
        # Speed histogram
        if len(self.speed_history) > 0:
            speeds = np.array(self.speed_history)
            self.ax_hist.hist(speeds, bins=20, color='green', alpha=0.6, edgecolor='black')
            self.ax_hist.axvline(x=np.mean(speeds), color='r', linestyle='--', linewidth=2, label=f'Mean: {np.mean(speeds):.2f}')
        
        self.ax_hist.set_title('Speed Distribution')
        self.ax_hist.set_xlabel('Speed (m/s)')
        self.ax_hist.set_ylabel('Frequency')
        self.ax_hist.grid(True, alpha=0.3)
        self.ax_hist.legend(loc='upper right', fontsize=8)
        
        # Lateral error histogram
        if len(self.lateral_error_history) > 0:
            errors = np.array(self.lateral_error_history)
            self.ax_error_hist.hist(errors, bins=20, color='red', alpha=0.6, edgecolor='black')
            self.ax_error_hist.axvline(x=0, color='k', linestyle='--', alpha=0.3)
            self.ax_error_hist.axvline(x=np.mean(errors), color='b', linestyle='--', linewidth=2, label=f'Mean: {np.mean(errors):.3f}')
        
        self.ax_error_hist.set_title('Lateral Error Distribution')
        self.ax_error_hist.set_xlabel('Lateral Error (m)')
        self.ax_error_hist.set_ylabel('Frequency')
        self.ax_error_hist.grid(True, alpha=0.3)
        self.ax_error_hist.legend(loc='upper right', fontsize=8)
        
        plt.tight_layout()
        # Force redraw (but don't flush events too aggressively)
        try:
            self.fig.canvas.draw()
            # Only flush events occasionally to avoid blocking other GUIs
            # Use a counter to limit flush frequency
            if not hasattr(self, '_flush_counter'):
                self._flush_counter = 0
            self._flush_counter += 1
            if self._flush_counter % 5 == 0:  # Flush every 5 updates
                self.fig.canvas.flush_events()
        except Exception as e:
            # Silently ignore GUI errors to avoid blocking
            pass


def main(args=None):
    rclpy.init(args=args)
    node = RealtimeVisualizer()
    
    try:
        # Use spin_once in a loop to allow matplotlib to update
        import time
        pause_counter = 0
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            # Reduce matplotlib event processing frequency to avoid blocking other GUIs
            # Only pause occasionally (every 10 iterations = ~1 second)
            pause_counter += 1
            if pause_counter >= 10:
                try:
                    plt.pause(0.01)  # Small pause for GUI events (reduced frequency)
                except Exception:
                    pass
                pause_counter = 0
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        plt.close('all')  # Close all matplotlib figures


if __name__ == '__main__':
    main()
