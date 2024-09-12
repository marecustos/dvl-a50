#!/usr/bin/env python3

import rclpy
import os
import re
import math
import time
from rclpy.node import Node
from pymavlink import mavutil
from dvl_msgs.msg import DVLDR

class DVLPositionSubscriberGPSDataSender(Node):
    def __init__(self):
        super().__init__('dvl_position_subscriber_gps_data_sender')

        # MAVLink connections
        self.symbolic_link = '/dev/pixhawk'
        self.real_device_path = self.get_real_device_path(self.symbolic_link)
        self.device_candidates = self.get_device_candidates(self.real_device_path)

        #think about replacing this communication master by the companion_rx in the buttom , it should works but it still needs to be tested
        self.master_comm = self.create_mavlink_connection(self.device_candidates)

        # DVL position subscriber
        self.subscription = self.create_subscription(
            DVLDR,
            '/dvl/position',
            self.listener_callback,
            10
        )

        # Timer to periodically check for LOCAL_POSITION_NED messages
        self.timer = self.create_timer(0.1, self.check_local_position_ned)

        # Initialize variables for GPS calculation
        self.current_lat = 31.5
        self.current_lon = 34.4667
        self.initial_lat = self.current_lat
        self.initial_lon = self.current_lon
        self.eph = 300  # Horizontal dilution of precision
        self.epv = 300  # Vertical dilution of precision

        # MAVLink connection to QGroundControl (or similar)
        self.qgc_tx = mavutil.mavlink_connection('udpout:base-station.local:14550', dialect="ardupilotmega", source_component=1, source_system=1)
        self.companion_rx = mavutil.mavlink_connection('udp:seabot-companion.local:14550')

    def get_real_device_path(self, symbolic_link):
        """Get the real path of a symbolic link."""
        try:
            real_path = os.readlink(symbolic_link)
            self.get_logger().info(f"real path {os.path.join('/dev', real_path)}")
            return self.decrement_device_number(os.path.join('/dev', real_path))
        except OSError as e:
            self.get_logger().error(f"Error reading symbolic link: {e}")
            return None

    def get_device_candidates(self,real_device_path):
        """Generate a list of device paths around the real path of /dev/pixhawk."""
        match = re.match(r'^(.+?)(\d+)$', real_device_path)
        if match:
            base_path = match.group(1)
            number = int(match.group(2))
            # Create a list with current device, number-1, and number+1
            return [f"{base_path}{number-1}", f"{base_path}{number}", f"{base_path}{number+1}"]
        return []

    def is_pixhawk_device(self,device_path):
        """Check if the given device is Pixhawk by waiting for a MAVLink heartbeat."""
        try:
            connection = mavutil.mavlink_connection(device_path, baud=115200)
            connection.wait_heartbeat(timeout=5)
            print(f"Found Pixhawk on {device_path}")
            return True
        except Exception as e:
            print(f"Device {device_path} is not Pixhawk. Error: {e}")
            return False

    def create_mavlink_connection(self,device_candidates):
        """Attempt to create a MAVLink connection using a list of device candidates."""
        for device_path in device_candidates:
            if self.is_pixhawk_device(device_path):
                try:
                    connection = mavutil.mavlink_connection(device_path, baud=115200)
                    connection.wait_heartbeat()
                    print(f"Connected to Pixhawk on {device_path}")
                    return connection
                except Exception as e:
                    print(f"Failed to connect to {device_path}. Error: {e}")
        print("Could not establish a connection to any Pixhawk device.")
        return None

    def decrement_device_number(self, device_path):
        """Increment the device number in the device path."""
        match = re.match(r'^(.+?)(\d+)$', device_path)
        if match:
            base_path = match.group(1)
            number = int(match.group(2))
            new_number = number - 1
            return f"{base_path}{new_number}"
        else:
            self.get_logger().error("Device path does not match expected pattern.")
            return None

    def listener_callback(self, msg: DVLDR):
        """Send VISION_POSITION_ESTIMATE to Pixhawk."""
        timestamp = self.get_clock().now().nanoseconds / 1e9
        self.master_comm.mav.vision_position_estimate_send(
            int(timestamp * 1e6),  # Timestamp in microseconds
            msg.position.x,         # X position
            msg.position.y,         # Y position
            msg.position.z,         # Z position
            msg.roll,               # Roll
            msg.pitch,              # Pitch
            msg.yaw                 # Yaw
        )

    def check_local_position_ned(self):
        """Check for LOCAL_POSITION_NED messages and use them to mock GPS data."""
        msg = self.companion_rx.recv_match(type='LOCAL_POSITION_NED', blocking=False)
        if msg:
            # Extract x, y, z position values
            local_x = msg.x
            local_y = msg.y
            local_z = msg.z

            # Calculate new GPS coordinates based on x and y
            new_lat, new_lon = self.calculate_new_gps(self.initial_lat, self.initial_lon, local_x, local_y)
            self.current_lat = new_lat
            self.current_lon = new_lon

            # Send the raw GPS data using GPS_RAW_INT
            self.qgc_tx.mav.gps_raw_int_send(
                int(time.time() * 1e6),  # time_usec: Unix epoch time in microseconds
                3,  # fix_type: 3D fix
                int(new_lat * 1e7),  # latitude in degrees * 1e7
                int(new_lon * 1e7),  # longitude in degrees * 1e7
                int(local_z * 1000),  # altitude in millimeters (MSL)
                self.eph,  # HDOP: Horizontal dilution of precision
                self.epv,  # VDOP: Vertical dilution of precision
                0,  # GPS ground speed (set to 0 if unknown)
                0,  # Course over ground (set to 0 if unknown)
                8   # Number of satellites visible (example value, adjust as needed)
            )

    def calculate_new_gps(self, lat, lon, x_offset, y_offset):
        """Calculate new GPS coordinates based on x and y offsets."""
        # Earth's radius in meters
        earth_radius = 6378137.0
        
        # Coordinate offsets in radians
        d_lat = y_offset / earth_radius
        d_lon = x_offset / (earth_radius * math.cos(math.pi * lat / 180.0))
        
        # New coordinates in degrees
        new_lat = lat + (d_lat * 180.0 / math.pi)
        new_lon = lon + (d_lon * 180.0 / math.pi)
        
        return new_lat, new_lon


def main(args=None):
    rclpy.init(args=args)
    node = DVLPositionSubscriberGPSDataSender()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down...")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
