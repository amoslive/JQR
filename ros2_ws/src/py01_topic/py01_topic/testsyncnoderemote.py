import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
# from base_interfaces_demo.msg import MotorCommand, ImuMsg, RemoteMsg
from .testsync1 import CANopenMotorController
#from .processremote import UBTRemoteMaster, UBTRemoteData
import time
import math
import csv
import os
from datetime import datetime

class MotorControllerNode(Node):
    Vel_Mode = 3  # Example: Velocity3, Position7, Torque4
    Pos_Mode = 7
    Toq_Mode = 4
    def __init__(self):
        super().__init__('motor_controller_node')

        self.status_publisher = self.create_publisher(
            Float64MultiArray,
            'waist_motor_status_array',   # target topic name
            10)

        # self.imu_subscription = self.create_subscription(
        #     ImuMsg,
        #     'imu_msg',
        #     self.listener_callback1,
        #     10)

        # self.motor_command_subscription = self.create_subscription(
        #     MotorCommand,
        #     'motor_command_sin',
        #     self.listener_callback2,
        #     10)

        # Add subscription for remote channel data
        # self.remote_subscription = self.create_subscription(
        #     RemoteMsg,
        #     'remote_msg',
        #     self.listener_callback3,
        #     1
        # )
        # Add subscription for waist motor command data
        self.motor_cmd_sub = self.create_subscription(
            Float64MultiArray,          
            'motor_cmd_array',          
            self.listener_callback4,    
            1                          
        )
        self.motor_controller = CANopenMotorController()

        # Initialize IMU data
        self.imu = [0, 0, 0]  
        # Initialize operation modes and target values
        self.operation_mode = 3  # Example: Velocity3, Position7, Torque4
        self.previous_operation_mode = 3
        self.target_velocity = 0
        self.target_torque = 0
        self.target_position = 0  

        # Prepare motor for operation (adjusted: no motor parameter)
        self.prepare_motor()

        # Start time for sinusoidal torque on Motor 3
        self.K = 0  # Initialize K

        # Create a timer for logging every 100 ms
        #self.log_timer = self.create_timer(0.1, self.log_motor_status)
        # Create a timer for logging every 5 ms
        self.log_timer = self.create_timer(0.005, self.log_motor_status)
        # Create a timer for updating motor targets every 5 ms
        self.update_timer = self.create_timer(0.005, self.update_motor_targets)

        # self.remote_master = UBTRemoteMaster()  # Instance of remote data processor
        # self.processed_data = UBTRemoteData()  # Store processed remote data

        # Create a CSV log file
        log_dir = os.path.join(os.getcwd(), 'waist_motor_data')
        os.makedirs(log_dir, exist_ok=True)
        base_name = datetime.now().strftime('%Y%m%d-%H%M%S')
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
        self.writer.writerow(['elapsed_time', 'target_velocity', 'velocity',
                                  'target_torque', 'current', 'target_position', 'position'])
        self.get_logger().info(f"Log file: {self.file_path}")

    def prepare_motor(self):
        # Call with operation mode only
        self.motor_controller.prepare_motor_run(self.operation_mode)

    # def listener_callback1(self, msg: ImuMsg):
    #     # self.get_logger().info(
    #     #     f'Received IMU: roll {msg.roll}, pitch {msg.pitch}, yaw {msg.yaw}')

    #     # Update IMU data 
    #     self.imu[0] = msg.roll
    #     self.imu[1] = msg.pitch
    #     self.imu[2] = msg.yaw

    # def listener_callback2(self, msg: MotorCommand):
    #     pass
        # self.get_logger().info(
        #     f'Received command: Velocity {msg.velocity_command}, Position {msg.position_command}')
        
        # Update target values
        #self.target_velocity = msg.velocity_command
        #self.target_position = msg.position_command  

    # def listener_callback3(self, msg: RemoteMsg):   
    #     """
    #     Process remote channel data and store it in self.processed_data.
    #     """
    #     channels = msg.channels
    #     self.processed_data = self.remote_master.get_remote_value(channels)  

    def listener_callback4(self, msg: Float64MultiArray):
        print("callback 4......")
        mode2 = msg.data[6]
        pos2 = msg.data[7]
        vel2 = msg.data[8]
        tff2 = msg.data[9]
        self.get_logger().info(
            f'Received command: Mode {mode2}, Torque {tff2}, Velocity {vel2}, Position {pos2}')

        # Update target values
        self.operation_mode = mode2
        # self.target_velocity = vel2
        # self.target_position = pos2 
        # self.target_torque = 0
        #self.target_velocity = vel2 * 62144 *0.5
        #self.target_velocity = vel2 
        #self.target_position = 0
        #self.target_position = pos2 - 2722385  
        self.target_position = 0.0
        self.target_velocity = 0.0
        self.target_torque = 0.0


    def update_motor_targets(self):
        # Send SYNC signal
        self.motor_controller.network.sync.transmit()
        
        #elapsed_time = 0.005 * self.K
        elapsed_time = round(0.005 * self.K, 3)
        self.K += 1  # Increment counter
  
        # self.operation_mode = self.Pos_Mode  # fixed position mode，adjust as needed  Velocity3, Position7, Torque4
        # self.operation_mode = self.Vel_Mode  # fixed velocity mode，adjust as needed  Velocity3, Position7, Torque4
        #self.operation_mode = self.Toq_Mode  # fixed torque mode，adjust as needed  Velocity3, Position7, Torque4
        # Example uses fixed velocity; sine-wave generation is disabled
        #float_velocity = 6000
        #self.target_velocity = int(float_velocity)

        # amplitude_vel = 62140
        # frequency = 0.5
        # (Legacy) sine-wave command transmission disabled below
        # self.target_velocity = amplitude_vel * math.sin(2 * math.pi * frequency * elapsed_time)

        #self.operation_mode = self.Toq_Mode
        # Note: 15 is approx. 1 Nm (15/1000*30A*2.2Nm/A)
        #amplitude_torque = 45
        #self.target_torque = amplitude_torque * math.sin(2 * math.pi * frequency * elapsed_time)

        # amplitude_pos = 1000
        # self.target_position = amplitude_pos * math.sin(2 * math.pi * frequency * elapsed_time)
        # position sine wave
        # self.operation_mode = Pos_Mode
        # Note: 15 is approx. 1 Nm (15/1000*30A*2.2Nm/A)
        # amplitude_pos = 30000 * 5 
        # self.target_position = amplitude_pos * math.sin(2 * math.pi * frequency * elapsed_time)
        
        #self.target_torque = -1600
        # Send a step command for several control cycles
        # if self.K <  100: 
        #      self.target_torque = -1600
        # elif self.K <  200:
        #      self.target_torque = -1600
        # else :
        #      self.target_torque = 0
        # else :
        #self.operation_mode = Pos_Mode  # fixed position mode
        #self.target_position = 0

        


        ##########   Test Start  ##########
        #self.target_velocity = 0
        mode, current, velocity, position = self.motor_controller.get_motor_status()
        #self.target_position = position
        #print(f"Actual Position: {position}")
        #=========================================================#
        # if position <  -680292: 
        #     self.target_torque = 0
        #     #print("Actual Position > 0")
        # else:
        #     self.target_torque = -500
        #     #print("Actual Position < 0")
        #=========================================================#
        ##########   Test End  ##########
        # Optionally enable sine-wave command generation, e.g.:
        # amplitude = 62144
        # frequency = 0.5
        # self.target_position = amplitude * math.sin(2 * math.pi * frequency * elapsed_time)
        #self.target_position = -211047
        #
        # amplitude = 400
        # frequency = 0.5
        # float_torque = amplitude * math.sin(2 * math.pi * frequency * elapsed_time)
        # self.target_torque = int(float_torque)
        # Manual run or via remote switch; clear errors if present
        ## can_status_wstrs is user-defined
        # if self.can_status_wstrs[0] == "fault":
        #             motor = self.can_motor_controller.motors[0]
        #             motor.rpdo[1]['controlword'].raw = 0x0000
        #             motor.rpdo[1].transmit()
        #             time.sleep(0.005)
        #             self.can_motor_controller.network.sync.transmit()
                    
        #             motor.rpdo[1]['controlword'].raw = 0x0080
        #             motor.rpdo[1].transmit()
        #             time.sleep(0.005)
        #             self.can_motor_controller.network.sync.transmit()
        #             self.can_motor_controller.prepare_motor_run(motor, self.can_operation_modes[0])
        #             time.sleep(0.005)
        #             print(f"Send clear fault control word to Motor {motor.id}")

        # If the mode changes, call change_motor_mode (motor argument removed)
        if self.operation_mode != self.previous_operation_mode:
            self.motor_controller.change_motor_mode(
                self.operation_mode,
                self.target_velocity,
                self.target_torque,
                self.target_position)
            self.previous_operation_mode = self.operation_mode

        # Enable motor run command (motor argument removed)
        self.motor_controller.run_motor(
            self.target_velocity,
            self.target_torque,
            self.target_position)

        

        # Publish measured motor state message
        mode, current, velocity, position = self.motor_controller.get_motor_status()
        msg = Float64MultiArray()
        msg.data = [float(position), float(velocity), float(current)]  # Note: the order must match the C++ receiver
        self.status_publisher.publish(msg)

        now = datetime.now().isoformat()
        self.writer.writerow([elapsed_time, self.target_velocity, velocity, self.target_torque, current, self.target_position, position])
 


    def log_motor_status(self):
        # Read status; motor handle stored internally
        mode, current, velocity, position = self.motor_controller.get_motor_status()

        self.get_logger().info(
           f"Motor status, Mode: {mode}, Target Velocity: {self.target_velocity}, "
           f"Velocity: {velocity}, Target Torque: {self.target_torque}, Current: {current}, "
           f"Target Position: {self.target_position}, Position: {position}")

    def stop_motors(self):
        self.motor_controller.stop_motor()
        self.get_logger().info('Motor stopped.')

    def destroy_node(self):
        self.stop_motors()
        if hasattr(self, 'csv_file'):
            self.csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    motor_controller_node = MotorControllerNode()

    try:
        rclpy.spin(motor_controller_node)
    except KeyboardInterrupt:
        pass
    finally:
        motor_controller_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
