import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float64MultiArray

class RemoteControllerListener(Node):
    def __init__(self):
        super().__init__('remote_controller_listener')

        # Subscribe to 'channel_values' (remote controller channels)
        self.subscriber = self.create_subscription(
            Float32MultiArray,  # message type
            'channel_values',   # topic name
            self.channel_callback,  # callback
            10
        )
        
        # Publisher for 'motor_cmd_array'
        self.motor_cmd_pub = self.create_publisher(Float64MultiArray, 'motor_cmd_array', 10)
        
        # Log startup message
        self.get_logger().info("Remote controller listener started; waiting for input...")

    def channel_callback(self, msg):
        # Extract channel data
        channel_values = msg.data

        if len(channel_values) != 8:
            self.get_logger().warn(f"Invalid data received: channel count is {len(channel_values)} (expected 8).")
            return

        # Parse channels using the configured mapping
        left_joystick_up_down = channel_values[1]  # Left joystick up/down (mapped to CH1)
        right_joystick_up_down = channel_values[2]  # Right joystick up/down (mapped to CH2)

        # Log and process channel values
        self.get_logger().info(f"Left joystick up/down (CH1): {left_joystick_up_down}")
        self.get_logger().info(f"Right joystick up/down (CH2): {right_joystick_up_down}")

        # Map left joystick (up/down) to a target wheel speed dq
        if 574 <= left_joystick_up_down < 740:
            dq = (left_joystick_up_down - 740) / (740 - 574) * 21 * 9.1  # negative speed: -(21 * 9.1) rad/s to 0
        elif 740 <= left_joystick_up_down <= 760:
            dq = 0  # stop
        elif 760 < left_joystick_up_down <= 924:
            dq = (left_joystick_up_down - 760) / (924 - 760) * 21 * 9.1  # positive speed: 0 to (21 * 9.1) rad/s
        else:
            dq = 0  # out-of-range -> 0

        # Log dq
        self.get_logger().info(f"Computed dq (speed): {dq}")

        # Map right joystick (up/down) to a target waist position pos2
        if 499 <= right_joystick_up_down <= 1000:
            # pos2 = (right_joystick_up_down - 499) / (1000 - 499) * 210000  # 映射到 pos2，范围 0 到 210000
            pos2 = int(round((right_joystick_up_down - 499) / (1000 - 499) * 210000))  # map to pos2 in [0, 210000]
        else:
            pos2 = 0  # out-of-range -> 0

        # Log pos2
        self.get_logger().info(f"Computed pos2 (position): {pos2}")

        # Build motor command message from remote inputs
        motor_cmd = Float64MultiArray()
        motor_cmd.data = [
            float(10.0),       # rear motor mode (constant placeholder, e.g., FOC)
            float(0.0),        # q (position reference)
            float(dq),         # dq (velocity reference, mapped)
            float(0.0),        # tau (set to 0 here)
            float(0.0),        # kp (default)
            float(3.0),        # kd (default)

            float(7.0),        # waist motor mode (constant placeholder, e.g., Position)
            float(pos2),       # pos2 (mapped from right joystick)
            float(0.0),        # vel2 (unused)
            float(0.0)         # tff2 (feedforward, set to 0)
        ]

        # Publish to 'motor_cmd_array'
        self.motor_cmd_pub.publish(motor_cmd)

        self.get_logger().info(f"Published motor_cmd_array: {motor_cmd.data}")


def main(args=None):
    rclpy.init(args=args)
    node = RemoteControllerListener()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
