# canopen_motor_controller.py

import canopen
import time
import os

class CANopenMotorController:
    def __init__(self, channel='can0', bustype='socketcan'):
        self.network = canopen.Network()
        self.network.connect(bustype=bustype, channel=channel)
        current_directory = os.path.dirname(os.path.abspath(__file__))
        eds_file_path = os.path.join(current_directory, 'CANedsGoldV002.eds')

        self.motor = canopen.RemoteNode(1, eds_file_path)
        self.network.add_node(self.motor)
        self.configure_pdo(self.motor)

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

    def prepare_motor_run(self, mode):
        motor = self.motor
        motor.rpdo[1]['controlword'].raw = 0x0006
        motor.rpdo[1].transmit()
        time.sleep(0.005)
        self.network.sync.transmit()
        
        self.init_motor_mode(mode)

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

    def init_motor_mode(self, mode):
        motor = self.motor
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

    def change_motor_mode(self, mode, target_velocity, target_torque, target_position):
        motor = self.motor
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

    def run_motor(self, target_velocity, target_torque, target_position):
        motor = self.motor
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

    def get_motor_status(self):
        """Retrieve the current status of the motor."""
        motor = self.motor
        try:
            mode = motor.tpdo[1]['modes_of_operation_display'].phys
            current = motor.tpdo[4]['torque_actual_value'].phys
            velocity = motor.tpdo[4]['velocity_actual_value'].phys
            position = motor.tpdo[1]['position_actual_value'].phys
        except Exception as e:
            print(f"Error reading status from Motor {motor.id}: {e}")
            return None, None, None, None
        
        return mode, current, velocity, position

    def stop_motor(self):
        motor = self.motor
        motor.rpdo[1]['controlword'].raw = 0x0000
        motor.rpdo[1].transmit()
        time.sleep(0.005)
        self.network.sync.transmit()
        print(f"Motor {motor.id} stopped")

    def disconnect(self):
        self.network.disconnect()
