import time
import math
import numpy as np
from dataclasses import dataclass, is_dataclass, field, fields
from typing import Any, List
import math
from enum import Enum, auto
import copy
import canopen
import os
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory


from std_msgs.msg import Float64MultiArray
from waistcar_msgs.msg import RemoteMsg

MVEL = 3
MTOR = 4
MPOS = 7

NUM_MOTOR = 1

DEGREE2RADIAN = math.pi / 180.0
RPM2RADPS = math.pi / 30.0

class CanStatus(Enum):
    CAN_INIT = 1
    CAN_RUN  = 2
    def __str__(self):
        return self.name

class MotorCmd:
    def __init__(self, cmd_mode, cmd_pos, cmd_vel, cmd_tor):
        self.cmd_mode = cmd_mode
        self.cmd_pos = cmd_pos  # position command
        self.cmd_vel = cmd_vel  # velocity command
        self.cmd_tor = cmd_tor

    def __repr__(self):
        return f"MotorCmd(cmd_mode={self.cmd_mode}, cmd_pos={self.cmd_pos}, cmd_vel={self.cmd_vel}, cmd_tor={self.cmd_tor})"

class MotorState:
    def __init__(self, act_mode, act_pos, act_vel, act_tor):
        self.act_mode = act_mode
        self.act_pos  = act_pos  # position command
        self.act_vel  = act_vel  # velocity command
        self.act_tor  = act_tor
    
    def __repr__(self):
        return f"MotorState(act_mode={self.act_mode}, act_pos={self.act_pos}, act_vel={self.act_vel}, act_tor={self.act_tor})"


class MotorControllerNode(Node):
    def __init__(self):
        super().__init__('canopen_motor_node')

        self.network = canopen.Network()
        self.network.connect(bustype='socketcan', channel='can0')
        # pkg_share_path = get_package_share_directory('py01_topic')
        # eds_file_path = os.path.join(pkg_share_path, 'CANedsGoldV002.eds')
        pkg_share = get_package_share_directory('py01_topic')
        eds_file_path = os.path.join(pkg_share, 'eds', 'CANedsGoldV002.eds')

        #############  by origin  #####################
        # current_directory = os.path.dirname(os.path.abspath(__file__))
        # eds_file_path = os.path.join(current_directory, 'CANedsGoldV002.eds')
        
        self.motors = [
            canopen.RemoteNode(1, eds_file_path),
        ]
        
        for motor in self.motors:
            self.network.add_node(motor)
            self.configure_pdo(motor)

        self.remote_cmd = RemoteMsg()
        self.last_remote_cmd = RemoteMsg()
        self.motor_cmd_msg = Float64MultiArray()
        self.motor_state_msg = Float64MultiArray()
        self.motor_cmd = [MotorCmd(MVEL,0,0,0)]
        self.last_motor_cmd = copy.deepcopy(self.motor_cmd)
        self.motor_state = [MotorState(0,0,0,0)]

        # CANopen initialization
        time.sleep(0.005)
        for i, motor in enumerate(self.motors):
            self.motor_cmd[i].cmd_mode = MVEL  # default mode: velocity control
            self.prepare_motor_run(motor, self.motor_cmd[i].cmd_mode)
            time.sleep(0.005)

        # Motor parameters
        self.coe_cntpercycle = np.array([65536.0])
        self.coe_trans = np.array([-12.0])
        self.coe_loadcntperrad = self.coe_cntpercycle * self.coe_trans / math.pi / 2
        self.coe_NmperA = np.array([0.1833])
        self.coe_maxA = np.array([91.0])
        self.coe_loadmaxNm = self.coe_maxA * self.coe_NmperA * self.coe_trans
        self.coe_loadcmdAperNm = 1.0 /self.coe_NmperA / self.coe_trans / self.coe_maxA * 1000.0
        print(f"coe_loadcmdAperNm: {self.coe_loadcmdAperNm[0]:.3f}, coe_loadcntperrad: {self.coe_loadcntperrad[0]:.3f}")

        self.operation_mode = MVEL  # Example: Velocity3, Position7, Torque4
        self.previous_operation_mode = 3
        self.target_velocity = 0
        self.target_torque = 0
        self.target_position = 0  
        
        self.can_status_words = np.zeros(NUM_MOTOR, dtype=int)
        self.can_status_wstrs = ["unknown"]

        # CANopen periodic thread
        self.can_dt = 0.005
        self.can_comm_timer = self.create_timer(self.can_dt, self.timer_callback_can_tr)  # 5ms timer
        self.can_status=CanStatus.CAN_INIT
        self.last_can_status = CanStatus.CAN_INIT

        # CAN monitoring: fault detection
        self.has_can_fault = False 

        # CANopen command timeout monitoring
        self.timer = self.create_timer(
            0.1,  # timer period (s)
            self.timer_callback_check_timeout
        )

        self.log_timer = self.create_timer(
            1, 
            self.log_motor_status)


        self.motor_publisher = self.create_publisher(Float64MultiArray, 'waist_motor_status_array', 1) 
        self.motor_subscriber = self.create_subscription(Float64MultiArray, 'motor_cmd_array', self.motor_cmd_callback, 1)
        self.remote_subscriber = self.create_subscription(
            RemoteMsg,  # message type
            'remote_topic',   # topic name
            self.remote_callback,  # callback
            1
        )

        self.last_receive_time = time.monotonic()
        self.timeout_cnt = 0

    def remote_callback(self, msg:RemoteMsg):
        self.remote_cmd = msg
    
    def log_motor_status(self):
        # Read status; motor handle stored internally
        print(f"cmd:{self.motor_cmd[0]}")
        print(f"state:{self.motor_state[0]}")
        print(f"can_status_wstrs:{self.can_status_wstrs[0]}")

    def motor_cmd_callback(self, msg:Float64MultiArray):
        self.last_receive_time = time.monotonic()
        # print("callback 4......")
        mode2 = msg.data[6]
        pos2 = msg.data[7]
        vel2 = msg.data[8]
        tff2 = msg.data[9]
        # self.get_logger().info(
        #     f'Received command: Mode {mode2}, Torque {tff2}, Velocity {vel2}, Position {pos2}')
        # Update target values
        self.motor_cmd[0].cmd_mode = mode2
        self.motor_cmd[0].cmd_pos = pos2
        self.motor_cmd[0].cmd_vel = vel2
        self.motor_cmd[0].cmd_tor = tff2

        # if self.motor_cmd[0].cmd_mode == MTOR:
        #     print(f"cmd_tor:{self.motor_cmd[0].cmd_tor:.2f}")

        if self.motor_cmd[0].cmd_mode == 0:
            self.motor_cmd[0].cmd_mode = MTOR
            self.motor_cmd[0].cmd_pos = 0
            self.motor_cmd[0].cmd_vel = 0
            self.motor_cmd[0].cmd_tor = 0
            # self.stop_motor(self.motors[0])

    def timer_callback_check_timeout(self):
        current_time = time.monotonic()
        if current_time - self.last_receive_time > 0.1:  # 0.1 s timeout
            pass
            for i in range(NUM_MOTOR):
                self.motor_cmd[i].cmd_mode = MVEL
                self.motor_cmd[i].cmd_pos = 0.0
                self.motor_cmd[i].cmd_vel = 0.0
                self.motor_cmd[i].cmd_tor = 0.0
            self.timeout_cnt = self.timeout_cnt + 1
            if self.timeout_cnt % 20 == 0:
                self.get_logger().warn('Motor command timeout! Setting all motors to safe state (velocity mode, zero commands).')
        else:
            self.timeout_cnt = 0

    def timer_callback_can_tr(self):
        ##### send motor command
        self.network.sync.transmit()
        if self.can_status == CanStatus.CAN_INIT:
            # 
            if self.remote_cmd.left_side < -0.8 and self.last_remote_cmd.left_side >= -0.8:
            # if ((self.motor_cmd[0].cmd_mode == MPOS or self.motor_cmd[0].cmd_mode == MVEL 
            #     or self.motor_cmd[0].cmd_mode == MTOR) and 
            #     self.last_motor_cmd[0].cmd_mode != self.motor_cmd[0].cmd_mode):
                for i, motor in enumerate(self.motors):
                    if self.can_status_wstrs[i] == "fault":
                        motor.rpdo[1]['controlword'].raw = 0x0000
                        motor.rpdo[1].transmit()
                        time.sleep(0.005)
                        self.network.sync.transmit()
                        
                        motor.rpdo[1]['controlword'].raw = 0x0080
                        motor.rpdo[1].transmit()
                        time.sleep(0.005)
                        self.network.sync.transmit()
                        
                        self.prepare_motor_run(motor, self.motor_cmd[i].cmd_mode)
                        time.sleep(0.005)
                        print(f"Send clear fault control word to Motor {motor.id}")
            
            if (self.can_status_wstrs[0] != "fault"):
                self.can_status = CanStatus.CAN_RUN

        elif self.can_status == CanStatus.CAN_RUN:
            for i, motor in enumerate(self.motors):
                last_cmd_mode = self.last_motor_cmd[i].cmd_mode
                cmd_mode = self.motor_cmd[i].cmd_mode
                
                cmd_pos_raw = int(self.motor_cmd[i].cmd_pos * self.coe_loadcntperrad[i])
                cmd_vel_raw = int(self.motor_cmd[i].cmd_vel * self.coe_loadcntperrad[i])
                cmd_tor_raw = int(self.motor_cmd[i].cmd_tor * self.coe_loadcmdAperNm[i])
                # Check for mode changes and apply them if necessary
                if cmd_mode != last_cmd_mode:
                    self.change_motor_mode(
                        motor, cmd_mode,
                        cmd_vel_raw,
                        cmd_tor_raw,
                        cmd_pos_raw)
                # Update motor targets
                self.run_motor(
                    motor,
                    cmd_vel_raw,
                    cmd_tor_raw,
                    cmd_pos_raw)
                # self.run_motor(
                #     motor,
                #     0,
                #     0,
                #     0)
        else:
            pass
    
        ###### read motor status
        for i, motor in enumerate(self.motors):
            act_mode, act_tor_raw, act_vel_raw, act_pos_raw, can_status_word = (
                self.get_motor_status(motor)
            )
            self.motor_state[i].act_mode = act_mode
            self.motor_state[i].act_pos = (act_pos_raw / self.coe_loadcntperrad[i])
            self.motor_state[i].act_vel = (act_vel_raw / self.coe_loadcntperrad[i])
            self.motor_state[i].act_tor = (act_tor_raw / self.coe_loadcmdAperNm[i])
            self.motor_state[i].status_word = can_status_word
            self.can_status_words[i] = can_status_word
            if   self.can_status_words[i] & 0b01001111 == 0b00001000:
                 self.can_status_wstrs[i] = "fault"
            elif self.can_status_words[i] & 0b01001111 == 0b00001111:
                 self.can_status_wstrs[i] = "fault reaction active"
            elif self.can_status_words[i] & 0b01001111 == 0b00000000:
                 self.can_status_wstrs[i] = "not ready to swith on"
            elif self.can_status_words[i] & 0b01001111 == 0b01000000:
                 self.can_status_wstrs[i] = "swith on disabled"
            elif self.can_status_words[i] & 0b01101111 == 0b00100001:
                 self.can_status_wstrs[i] = "ready to switch on"
            elif self.can_status_words[i] & 0b01101111 == 0b00100011:
                 self.can_status_wstrs[i] = "switch on"
            elif self.can_status_words[i] & 0b01101111 == 0b00100111:
                 self.can_status_wstrs[i] = "operation enabled"
            elif self.can_status_words[i] & 0b01101111 == 0b00000111:
                 self.can_status_wstrs[i] = "quick stop active"
            else:
                pass
            self.motor_state[i].status_string = self.can_status_wstrs[i]
        # self.motor_state_msg.items = self.motor_state 
        self.motor_state_msg.data = [float(self.motor_state[0].act_pos), float(self.motor_state[0].act_vel), float(self.motor_state[0].act_tor)]  # Note: the order must match the C++ receiver
        self.motor_publisher.publish(self.motor_state_msg)

        # self.act_rear_tor = self.motor_state[REAR_IND].act_tor
        
        if self.can_status_wstrs[0] == "fault":
            self.can_status = CanStatus.CAN_INIT
            self.has_can_fault = True
        else:
            self.has_can_fault = False

        self.last_remote_cmd = copy.deepcopy(self.remote_cmd)
        self.last_can_status = copy.deepcopy(self.can_status)
        self.last_motor_cmd = copy.deepcopy(self.motor_cmd)
        # self.last_act_rear_tor = self.act_rear_tor

    def configure_pdo(self, motor):
        motor.nmt.state = 'PRE-OPERATIONAL'
        time.sleep(0.5)
        motor.tpdo.read()
        motor.rpdo.read()
        
        motor.sdo[0x60C2][1].write(5)
        time.sleep(0.5)
        
        motor.tpdo[1].clear()
        motor.tpdo[1].add_variable('statusword')
        motor.tpdo[1].add_variable('modes_of_operation_display')
        motor.tpdo[1].add_variable('position_actual_value')
        motor.tpdo[1].trans_type = 1
        motor.tpdo[1].enabled = True

        motor.tpdo[4].clear()
        motor.tpdo[4].add_variable('torque_actual_value')
        motor.tpdo[4].add_variable('velocity_actual_value')
        motor.tpdo[4].trans_type = 1
        motor.tpdo[4].enabled = True

        motor.tpdo[2].enabled = False
        motor.tpdo[3].enabled = False
        #motor.tpdo[1].enabled = False #test only rpdo
        #motor.tpdo[4].enabled = False #test only rpdo

        motor.rpdo[1].clear()
        motor.rpdo[1].add_variable('controlword')
        motor.rpdo[1].add_variable('modes_of_operation')
        motor.rpdo[1].add_variable('interpolation_data_record','interpolation_data_record_setpoint_1')
        motor.rpdo[1].trans_type = 1
        motor.rpdo[1].enabled = True

        motor.rpdo[2].clear()
        motor.rpdo[2].add_variable('target_torque')
        motor.rpdo[2].add_variable('target_velocity')
        motor.rpdo[2].trans_type = 1
        motor.rpdo[2].enabled = True
        
        motor.rpdo[3].enabled = False
        motor.rpdo[4].enabled = False
        
        motor.tpdo.save()
        motor.rpdo.save()
        
        motor.tpdo.read()
        motor.rpdo.read()    
        motor.nmt.state = 'OPERATIONAL'
        time.sleep(0.5)

    def send_sync(self):
        """Send SYNC signal"""
        self.network.sync.transmit()

    def prepare_motor_run(self, motor, mode):
        motor.rpdo[1]['controlword'].raw = 0x0006
        motor.rpdo[1].transmit()
        time.sleep(0.005)
        self.network.sync.transmit()
        
        self.init_motor_mode(motor, mode)

        motor.rpdo[1]['controlword'].raw = 0x0007
        motor.rpdo[1].transmit()
        time.sleep(0.005)
        self.network.sync.transmit()

        motor.rpdo[1]['controlword'].raw = 0x000F
        motor.rpdo[1].transmit()
        time.sleep(0.005)
        self.network.sync.transmit()

        if mode == 7:
            motor.rpdo[1]['controlword'].raw = 0x001F
            motor.rpdo[1].transmit()
            time.sleep(0.005)
            self.network.sync.transmit()
        print(f"Motor {motor.id} is ready, mode: {mode}")

    def init_motor_mode(self, motor, mode):
        if mode == 3:
            motor.rpdo[1]['modes_of_operation'].raw = 0x03
            motor.rpdo[1].transmit()
            time.sleep(0.005)
            self.network.sync.transmit()

            motor.rpdo[2]['target_velocity'].raw = 0
            motor.rpdo[2].transmit()
            time.sleep(0.005)
            self.network.sync.transmit()
            
        elif mode == 4:
            motor.rpdo[1]['modes_of_operation'].raw = 0x04
            motor.rpdo[1].transmit()
            time.sleep(0.005)
            self.network.sync.transmit()

            motor.rpdo[2]['target_torque'].raw = 0
            motor.rpdo[2].transmit()
            time.sleep(0.005)
            self.network.sync.transmit()
            
        elif mode == 7:
            motor.rpdo[1]['modes_of_operation'].raw = 0x07
            motor.rpdo[1]['interpolation_data_record.interpolation_data_record_setpoint_1'].raw = 0
            motor.rpdo[1].transmit()
            time.sleep(0.005)
            self.network.sync.transmit()
        print(f"Motor {motor.id} initial mode set to: {mode}")

    def change_motor_mode(self, motor, mode, target_velocity, target_torque, target_position):
        if mode == 3:
            motor.rpdo[1]['modes_of_operation'].raw = 0x03
            motor.rpdo[1].transmit()
            time.sleep(0.005)
            self.network.sync.transmit()

            motor.rpdo[2]['target_velocity'].raw = target_velocity
            motor.rpdo[2].transmit()
            time.sleep(0.005)
            self.network.sync.transmit()
        
        elif mode == 4:
            motor.rpdo[1]['modes_of_operation'].raw = 0x04
            motor.rpdo[1].transmit()
            time.sleep(0.005)
            self.network.sync.transmit()

            motor.rpdo[2]['target_torque'].raw = target_torque
            motor.rpdo[2].transmit()
            time.sleep(0.005)
            self.network.sync.transmit()

        elif mode == 7:
            motor.rpdo[1]['modes_of_operation'].raw = 0x07
            motor.rpdo[1]['interpolation_data_record.interpolation_data_record_setpoint_1'].raw = target_position
            motor.rpdo[1].transmit()
            time.sleep(0.005)
            self.network.sync.transmit()

            motor.rpdo[1]['controlword'].raw = 0x000F
            motor.rpdo[1].transmit()
            time.sleep(0.005)
            self.network.sync.transmit()
            motor.rpdo[1]['controlword'].raw = 0x001F
            motor.rpdo[1].transmit()
            time.sleep(0.005)
            self.network.sync.transmit()

        print(f"Motor {motor.id} run mode changed to: {mode}")

    def run_motor(self, motor, target_velocity, target_torque, target_position):
        motor.rpdo[2]['target_velocity'].raw = target_velocity
        motor.rpdo[2]['target_torque'].raw = target_torque
        motor.rpdo[1]['interpolation_data_record.interpolation_data_record_setpoint_1'].raw = target_position

        mode = motor.tpdo[1]['modes_of_operation_display'].phys
        current_actual = motor.tpdo[4]['torque_actual_value'].phys
        velocity_actual = motor.tpdo[4]['velocity_actual_value'].phys
        position_actual = motor.tpdo[1]['position_actual_value'].phys
        
        if mode == 3 or mode == 4:
            motor.rpdo[2].transmit()
        elif mode == 7:
            motor.rpdo[1].transmit()

        return mode, current_actual, velocity_actual, position_actual

    def get_motor_status(self, motor):
        """Retrieve the current status of the motor."""
        try:
            # Read motor state from process data objects (PDO)
            mode = motor.tpdo[1]['modes_of_operation_display'].phys
            current = motor.tpdo[4]['torque_actual_value'].phys
            velocity = motor.tpdo[4]['velocity_actual_value'].phys
            position = motor.tpdo[1]['position_actual_value'].phys
            statusword = motor.tpdo[1]['statusword'].phys
        except Exception as e:
            # Handle the case where data could not be read successfully
            print(f"Error reading status from Motor {motor.id}: {e}")
            return None, None, None, None
        
        return mode, current, velocity, position, statusword

    def stop_motor(self, motor):
        motor.rpdo[1]['controlword'].raw = 0x0000
        motor.rpdo[1].transmit()
        time.sleep(0.005)
        self.network.sync.transmit()
        print(f"Motor {motor.id} stopped")

    def disconnect(self):
        self.network.disconnect()

    def destroy_node(self):
        for motor in self.motors:
            self.stop_motor(motor)
        self.get_logger().info('Motors stopped.')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    motor_controller_node = MotorControllerNode()

    try:
        # Spin the node to handle callbacks and update motor targets
        rclpy.spin(motor_controller_node)
    except KeyboardInterrupt:
        pass
    finally:
        motor_controller_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
