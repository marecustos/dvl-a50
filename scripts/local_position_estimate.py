import rclpy
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
        self.real_device_path = self.get_real_device_path(self.symbolic_link)
        self.master_tx = self.create_mavlink_connection(self.real_device_path)

    def get_real_device_path(self):
    """Get the real path of a symbolic link."""
    try:
        real_path = os.readlink(self.symbolic_link)
        print(f"real path {os.path.join('/dev', real_path)}")
        return os.path.join('/dev', real_path)
    except OSError as e:
        print(f"Error reading symbolic link: {e}")
        return None

    def create_mavlink_connection(self,real_device_path):
        """Create a MAVLink connection using the base path and its incremented version."""
        connection = mavutil.mavlink_connection(real_device_path, baud=115200)

        return connection

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
