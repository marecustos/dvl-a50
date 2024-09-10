#!/usr/bin/env python3

import rclpy
import os
from rclpy.node import Node
from pymavlink import mavutil
from dvl_msgs.msg import DVLDR

class DVLPositionSubscriber(Node):
    def __init__(self):
        super().__init__('dvl_position_subscriber')
        self.subscription = self.create_subscription(
            DVLDR,
            '/dvl/position',
            self.listener_callback,
            10
        )
        self.symbolic_link = '/dev/pixhawk'
        self.real_device_path = self.get_real_device_path()
        self.master_tx = self.create_mavlink_connection()

    def get_real_device_path(self):
        """Get the real path of a symbolic link."""
        try:
            real_path = os.readlink(self.symbolic_link)
            print(f"real path {os.path.join('/dev', real_path)}")
            return increment_device_number(os.path.join('/dev', real_path))
        except OSError as e:
            print(f"Error reading symbolic link: {e}")
            return None

    def create_mavlink_connection(self):
        """Create a MAVLink connection using the base path and its incremented version."""
        connection = mavutil.mavlink_connection(self.real_device_path, baud=115200)

        return connection

    def increment_device_number(self, device_path):
        """Increment the device number in the device path."""
        # Match the pattern for the device (e.g., /dev/ttyACM0)
        match = re.match(r'^(.+?)(\d+)$', device_path)
        if match:
            base_path = match.group(1)
            number = int(match.group(2))
            new_number = number + 1
            return f"{base_path}{new_number}"
        else:
            print("Device path does not match expected pattern.")
            return None

    def listener_callback(self, msg: DVLDR):
        timestamp = self.get_clock().now().nanoseconds / 1e9

        self.master_tx.mav.vision_position_estimate_send(
            int(timestamp * 1e6),  # Timestamp in microseconds
            msg.position.x,         # X position
            msg.position.y,         # Y position
            msg.position.z,         # Z position
            msg.roll,               # Roll
            msg.pitch,              # Pitch
            msg.yaw                 # Yaw
        )

def main(args=None):
    rclpy.init(args=args)
    dvl_position_subscriber = DVLPositionSubscriber()

    try:
        rclpy.spin(dvl_position_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        dvl_position_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
