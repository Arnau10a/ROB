import os
import csv
import time
import subprocess
from datetime import datetime

class MissionLogger:
    def __init__(self, filename="mission_log.csv"):
        self.filename = filename
        self._init_csv()

    def _init_csv(self):
        """Initialize CSV file with header if it doesn't exist."""
        if not os.path.exists(self.filename):
            with open(self.filename, mode='w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    "timestamp", 
                    "current_phase", 
                    "robot_x", 
                    "robot_y", 
                    "robot_yaw", 
                    "obstacles", 
                    "station_pillars"
                ])

    def log(self, phase, x, y, yaw, obstacles=None, pillars=None):
        """
        Log a mission state entry.
        obstacles: list of (x, y) tuples
        pillars: list of (x, y) tuples
        """
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        obs_str = str(obstacles) if obstacles else "[]"
        pillars_str = str(pillars) if pillars else "[]"
        
        with open(self.filename, mode='a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                timestamp,
                phase,
                round(x, 3),
                round(y, 3),
                round(yaw, 3),
                obs_str,
                pillars_str
            ])

    def save_map(self, node, phase_name):
        """
        Save the current map using map_saver_cli.
        """
        map_filename = f"generated_map_{phase_name}"
        map_path = os.path.join(os.getcwd(), map_filename)
        
        node.get_logger().info(f'[MISSION_LOGGER] Requesting map save to {map_path}...')
        
        try:
            # We use a subprocess to call the ROS 2 map_saver_cli
            # This requires nav2_map_server to be installed
            cmd = [
                'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                '-f', map_path,
                '--ros-args', '-p', 'save_map_timeout:=5000'
            ]
            
            # Start process in background
            subprocess.Popen(cmd)
            node.get_logger().info(f'[MISSION_LOGGER] Map saver process started for {map_filename}')
        except Exception as e:
            node.get_logger().error(f'[MISSION_LOGGER] Failed to save map: {e}')
