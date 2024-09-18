#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import os
import time
import math

from rmd_x6_40_cntl.msg import Buttons
from rmd_x6_40_cntl.msg import MotorSpeed

pub = rospy.Publisher('gui_input', Buttons, queue_size=1)
pub_two = rospy.Publisher('motor_speed', MotorSpeed, queue_size=1)

os.system("sudo ip link set can0 down")
time.sleep(0.1)
os.system("sudo ip link set can0 up type can bitrate 1000000")

"""
Motor-specific configuration

144-----141
     |
     |
143-----142

"""
FRONT_LEFT_MOTOR_ID = 0x144
FRONT_RIGHT_MOTOR_ID = 0x141 
BACK_LEFT_MOTOR_ID = 0x143 
BACK_RIGHT_MOTOR_ID = 0x142 
# ALL_MOTOR_ID = 0x280

def speed_control_to_mps(speed_control, radius):
    """
    Convert speedControl value to linear velocity in meters per second (m/s).
    :param speed_control: The speed control value (int32_t).
    :param radius: Radius of the wheel or shaft in meters.
    :return: Linear velocity in meters per second (m/s).
    """
    # Step 1: Convert speedControl to Angular Velocity in dps
    angular_velocity_dps = speed_control * 0.01
    
    # Step 2: Convert Angular Velocity from dps to rad/s
    angular_velocity_rad_per_sec = angular_velocity_dps * (math.pi / 180)
    
    # Step 3: Convert Angular Velocity to Linear Speed in m/s
    linear_velocity = angular_velocity_rad_per_sec * radius

    # Step 4: Return the linear speed to 3 decimal places
    # return round(linear_velocity, 3)
    return linear_velocity

def send_speed_command(can_id, rpm):
    """
    Moves the motor.

    :param can_id: The motor CANID (integer).
    :param rpm: The desired motor speed in RPM (integer).
    :return: A string representing the 'cansend' command.
    """
    # Convert RPM to speedControl in dps (degrees per second)
    # 1 RPM = 6 degrees per second
    dps = rpm * 6

    # Convert to velocity (m/s)
    velocity = speed_control_to_mps(dps, 0.058)

    # Convert to the required units (0.01 dps/LSB)
    speed_control = int(dps * 100)  # 0.01 dps/LSB

    # If speed_control is negative, we need to convert it to 2's complement
    if speed_control < 0:
        speed_control = (1 << 32) + speed_control  # Two's complement for 32-bit

    # Convert speed_control to a 32-bit hexadecimal value
    speed_control_hex = f"{speed_control:08X}"

    # Split the 32-bit hexadecimal value into 4 bytes
    byte4 = speed_control_hex[0:2]  # High byte
    byte3 = speed_control_hex[2:4]
    byte2 = speed_control_hex[4:6]
    byte1 = speed_control_hex[6:8]  # Low byte

    # Construct the CAN data string
    can_data = f"A2 00 00 00 {byte1} {byte2} {byte3} {byte4}"

    # Create the 'cansend' command
    can_id_hex = f"{can_id:03X}"  # Convert CAN ID to 3-digit hex format
    cansend_command = f"cansend can0 {can_id_hex}#{can_data.replace(' ', '')}"

    # Send command to the CAN BUS
    os.system(cansend_command)
    print(f"Moving motor {can_id_hex} at {velocity} m/s.")

def get_position(id):
    """
    Stop the motor.

    :param id: The motor CANID (integer).
    """
    # Convert CAN ID to 3-digit hex format
    can_id_hex = f"{id:03X}"

    os.system(f"cansend can0 {can_id_hex}#9200000000000000")
    print(f"Aquiring motor {can_id_hex} pos")

def zero_position(id):
    """
    Zeros the current position of the motor.

    :param id: The motor CANID (integer).
    """
    # Convert CAN ID to 3-digit hex format
    can_id_hex = f"{id:03X}"

    os.system(f"cansend can0 {can_id_hex}#6400000000000000")
    print(f"Zeroing motor {can_id_hex}")
    os.system(f"cansend can0 {can_id_hex}#7600000000000000")

def send_stop_command(id):
    """
    Stop the motor.

    :param id: The motor CANID (integer).
    """
    # Convert CAN ID to 3-digit hex format
    can_id_hex = f"{id:03X}"

    os.system(f"cansend can0 {can_id_hex}#8100000000000000")
    print(f"Stopping motor {can_id_hex}")

def send_release_break_command(id):
    """
    Removes the break, placing the motor in motion control mode.

    :param id: The motor CANID (integer).
    """
    # Convert CAN ID to 3-digit hex format
    can_id_hex = f"{id:03X}"
    
    os.system(f"cansend can0 {can_id_hex}#7700000000000000")
    print(f"Releasing break from motor {can_id_hex}")

def send_activate_command(id):
    """
    Activates the motor.

    :param id: The motor CANID (integer).
    """
    # Convert CAN ID to 3-digit hex format
    can_id_hex = f"{id:03X}"
    
    os.system(f"cansend can0 {can_id_hex}#0100000000000000")
    print(f"Activating motor {can_id_hex}")

speed_limit = 90
prev_linear_x = 0.0
prev_angular_z = 0.0
speed = MotorSpeed()

def cmd_vel_callback(data):
    global prev_linear_x
    global prev_angular_z
    global speed
    
    linear_x = data.linear.x
    angular_z = data.angular.z 
    
    if linear_x != prev_linear_x:

        if linear_x != 0.0:

            linear_speed = linear_x * speed_limit
            send_speed_command(BACK_LEFT_MOTOR_ID, int(linear_speed))
            send_speed_command(FRONT_RIGHT_MOTOR_ID, int(linear_speed))
            send_speed_command(BACK_RIGHT_MOTOR_ID, int(linear_speed))
            send_speed_command(FRONT_LEFT_MOTOR_ID, int(linear_speed))
            
            get_position(BACK_LEFT_MOTOR_ID)
            get_position(FRONT_RIGHT_MOTOR_ID)
            get_position(BACK_RIGHT_MOTOR_ID)
            get_position(FRONT_LEFT_MOTOR_ID)
            
            speed.back_left_speed = int(linear_speed)
            speed.front_right_speed = int(linear_speed)
            speed.back_right_speed = int(linear_speed)
            speed.front_left_speed = int(linear_speed)

        else:
            send_stop_command(BACK_LEFT_MOTOR_ID)
            send_stop_command(FRONT_RIGHT_MOTOR_ID)
            send_stop_command(BACK_RIGHT_MOTOR_ID)
            send_stop_command(FRONT_LEFT_MOTOR_ID)
            
            get_position(BACK_LEFT_MOTOR_ID)
            get_position(FRONT_RIGHT_MOTOR_ID)
            get_position(BACK_RIGHT_MOTOR_ID)
            get_position(FRONT_LEFT_MOTOR_ID)
            
            speed.back_left_speed = 0
            speed.front_right_speed = 0
            speed.back_right_speed = 0
            speed.front_left_speed = 0

        pub_two.publish(speed)

    elif angular_z != prev_angular_z:
        
        angular_speed_towards = angular_z * speed_limit
        turning_speed_away = -1 * angular_speed_towards

        if angular_z < -0.02 or angular_z > 0.02:
            angular_speed_towards = angular_z * speed_limit
            turning_speed_away = -1 * angular_speed_towards

            send_speed_command(BACK_LEFT_MOTOR_ID, int(angular_speed_towards))
            send_speed_command(FRONT_RIGHT_MOTOR_ID, int(turning_speed_away))
            send_speed_command(BACK_RIGHT_MOTOR_ID, int(turning_speed_away))
            send_speed_command(FRONT_LEFT_MOTOR_ID, int(angular_speed_towards))
            
            get_position(BACK_LEFT_MOTOR_ID)
            get_position(FRONT_RIGHT_MOTOR_ID)
            get_position(BACK_RIGHT_MOTOR_ID)
            get_position(FRONT_LEFT_MOTOR_ID)

        else:
            send_stop_command(BACK_LEFT_MOTOR_ID)
            send_stop_command(FRONT_RIGHT_MOTOR_ID)
            send_stop_command(BACK_RIGHT_MOTOR_ID)
            send_stop_command(FRONT_LEFT_MOTOR_ID)
            
            get_position(BACK_LEFT_MOTOR_ID)
            get_position(FRONT_RIGHT_MOTOR_ID)
            get_position(BACK_RIGHT_MOTOR_ID)
            get_position(FRONT_LEFT_MOTOR_ID)

    else:
        get_position(BACK_LEFT_MOTOR_ID)
        get_position(FRONT_RIGHT_MOTOR_ID)
        get_position(BACK_RIGHT_MOTOR_ID)       
        get_position(FRONT_LEFT_MOTOR_ID)

    prev_linear_x = linear_x
    prev_angular_z = angular_z

def gui_input_callback(data):
    
    if data.zero == 1.0:
        send_stop_command(BACK_LEFT_MOTOR_ID)
        send_stop_command(FRONT_RIGHT_MOTOR_ID)
        send_stop_command(BACK_RIGHT_MOTOR_ID)
        send_stop_command(FRONT_LEFT_MOTOR_ID)
        
        get_position(BACK_LEFT_MOTOR_ID)
        get_position(FRONT_RIGHT_MOTOR_ID)
        get_position(BACK_RIGHT_MOTOR_ID)
        get_position(FRONT_LEFT_MOTOR_ID)


        zero_position(BACK_LEFT_MOTOR_ID)
        zero_position(FRONT_RIGHT_MOTOR_ID)
        zero_position(BACK_RIGHT_MOTOR_ID)
        zero_position(FRONT_LEFT_MOTOR_ID)
        
        get_position(BACK_LEFT_MOTOR_ID)
        get_position(FRONT_RIGHT_MOTOR_ID)
        get_position(BACK_RIGHT_MOTOR_ID)
        get_position(FRONT_LEFT_MOTOR_ID)

        reset = Buttons()
        reset.zero = 0.0
        pub.publish(reset)


def main():

    rospy.init_node('myactuator_cntl_node')

    rospy.Subscriber('cmd_vel', Twist, cmd_vel_callback)

    rospy.Subscriber('gui_input', Buttons, gui_input_callback)

    rospy.spin()

if __name__ == "__main__":

    main()
