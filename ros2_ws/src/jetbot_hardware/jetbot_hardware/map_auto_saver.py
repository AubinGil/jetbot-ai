#!/usr/bin/env python3
"""
Auto Map Saver for SLAM
Periodically saves the map being built by SLAM Toolbox
"""
import rclpy
from rclpy.node import Node
import subprocess
import os
from datetime import datetime

class MapAutoSaver(Node):
    def __init__(self):
        super().__init__('map_auto_saver')

        # Parameters
        self.declare_parameter('save_interval', 30.0)  # seconds
        self.declare_parameter('map_directory', os.path.expanduser('~/maps'))
        self.declare_parameter('map_name', 'jetbot_map')
        self.declare_parameter('keep_backups', True)

        self.save_interval = self.get_parameter('save_interval').value
        self.map_dir = self.get_parameter('map_directory').value
        self.map_name = self.get_parameter('map_name').value
        self.keep_backups = self.get_parameter('keep_backups').value

        # Create maps directory if it doesn't exist
        os.makedirs(self.map_dir, exist_ok=True)

        # Create timer for periodic saving
        self.timer = self.create_timer(self.save_interval, self.save_map_callback)

        self.save_count = 0
        self.get_logger().info(f'Map auto-saver started')
        self.get_logger().info(f'  Save interval: {self.save_interval}s')
        self.get_logger().info(f'  Map directory: {self.map_dir}')
        self.get_logger().info(f'  Map name: {self.map_name}')
        self.get_logger().info(f'  Keep backups: {self.keep_backups}')

    def save_map_callback(self):
        """Save the current map"""
        self.save_count += 1

        # Current map path (latest)
        current_map = os.path.join(self.map_dir, self.map_name)

        # Backup previous map if it exists
        if self.keep_backups and os.path.exists(f"{current_map}.pgm"):
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            backup_name = os.path.join(self.map_dir, f"{self.map_name}_backup_{timestamp}")

            try:
                # Backup both .pgm and .yaml files
                subprocess.run(['cp', f"{current_map}.pgm", f"{backup_name}.pgm"], check=True)
                subprocess.run(['cp', f"{current_map}.yaml", f"{backup_name}.yaml"], check=True)
                self.get_logger().info(f'Backed up previous map to {backup_name}')
            except subprocess.CalledProcessError as e:
                self.get_logger().warn(f'Failed to backup map: {e}')

        # Save current map
        try:
            self.get_logger().info(f'Saving map #{self.save_count} to {current_map}...')

            # Use map_saver_cli to save the map
            result = subprocess.run(
                ['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', current_map],
                capture_output=True,
                text=True,
                timeout=10
            )

            if result.returncode == 0:
                self.get_logger().info(f'✓ Map saved successfully (save #{self.save_count})')
            else:
                self.get_logger().error(f'Failed to save map: {result.stderr}')

        except subprocess.TimeoutExpired:
            self.get_logger().error('Map save timeout - is /map topic publishing?')
        except Exception as e:
            self.get_logger().error(f'Error saving map: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = MapAutoSaver()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
