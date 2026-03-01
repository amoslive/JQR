import rclpy
from rclpy.node import Node
import time
import csv
import os
from datetime import datetime
import sys
sys.path.append('../lib')
from unitree_actuator_sdk import *

from std_msgs.msg import Float64MultiArray

class MotorBridgeNode(Node):
    def __init__(self):
        super().__init__('motor_bridge_node')

        # Initialize serial port / actuator interface
        self.serial = SerialPort('/dev/com_motor')
        self.cmd = MotorCmd()
        self.data = MotorData()
        self.cmd.motorType = MotorType.A1
        self.cmd.mode = queryMotorMode(MotorType.A1, MotorMode.FOC)
        self.cmd.id = 1
        self.cmd.q = 0.0
        self.cmd.dq = 0.0
        self.cmd.kp = 0.0
        self.cmd.kd = 0.0
        self.cmd.tau = 0.0

        # Create CSV log file
        log_dir = os.path.join(os.getcwd(), 'motor_data')
        os.makedirs(log_dir, exist_ok=True)
        base_name = datetime.now().strftime('%Y%m%d_%H%M')
        counter = 0
        while True:
            suffix = f"_{counter}" if counter else ""
            filename = f"{base_name}{suffix}.csv"
            self.file_path = os.path.join(log_dir, filename)
            if not os.path.exists(self.file_path):
                break
            counter += 1
        self.csv_file = open(self.file_path, mode='w', newline='')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(['timestamp', 'q', 'dq', 'tau', 'merror'])
        self.get_logger().info(f"Log file: {self.file_path}")

        # ROS 2 interfaces
        self.sub = self.create_subscription(
            Float64MultiArray,
            'motor_cmd_array',
            self.command_callback,
            10
        )
        self.pub = self.create_publisher(
            Float64MultiArray,
            'motor_msg_array',
            10
        )

        # Timer (1000 Hz)
        self.tick = 0
        self.timer = self.create_timer(0.001, self.timer_callback)

    def command_callback(self, msg):
        try:
            if len(msg.data) >= 6:
                self.cmd.mode = int(msg.data[0])
                self.cmd.q = msg.data[1]
                self.cmd.dq = msg.data[2]
                self.cmd.tau = msg.data[3]
                self.cmd.kp = msg.data[4]
                self.cmd.kd = msg.data[5]
        except Exception as e:
            self.get_logger().error(f"Command parse error: {e}")

    def timer_callback(self):
        try:
            self.serial.sendRecv(self.cmd, self.data)
            now = datetime.now().isoformat()
            self.writer.writerow([now, self.data.q, self.data.dq, self.data.tau, self.data.merror])

            feedback = Float64MultiArray()
            feedback.data = [
                self.data.q,
                self.data.dq,
                self.data.tau
            ]
            self.pub.publish(feedback)

            # Optional: print feedback at ~10 Hz to avoid flooding the console
            # if self.tick % 100 == 0:
            #     self.get_logger().info(
            #         f"[Feedback] q: {self.data.q:.3f}, dq: {self.data.dq:.3f}, tau: {self.data.tau:.3f}"
            #     )
            # self.tick += 1

        except Exception as e:
            self.get_logger().error(f"Timer error: {e}")

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MotorBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Program interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
