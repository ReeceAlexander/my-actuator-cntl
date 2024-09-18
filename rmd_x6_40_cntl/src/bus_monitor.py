#!/usr/bin/env python3

import rospy
import subprocess
import re
import os
import time
import math
from rmd_x6_40_cntl.msg import MotorPosition

# Publish to the "/motor_feedback" topic
pub = rospy.Publisher('motor_position', MotorPosition, queue_size=4)
rospy.init_node('bus_monitor')
detected = MotorPosition()

def parse_motor_reply(can_message):
    """
    Extracts the motor position from a CAN message string and returns the absolute position in degrees.
    
    Parameters:
    can_message (str): A string representing the CAN message, e.g., "can0  244   [8]  92 00 00 00 A6 53 8E FF".
    
    Returns:
    float: The absolute position of the motor in degrees.
    """

    # Split the message string and extract the hexadecimal data bytes
    parts = can_message.split()
    # print(parts)
    can_id = int(parts[1])

    if can_id == 241 or can_id == 242 or can_id == 243 or can_id == 244:

        # print(len(parts))
        hex_data = parts[7:11]  # The data bytes are in the 7th to 11th positions in the split message
        # print(hex_data)
        
        # Convert the hex strings to integers
        can_data = [int(byte, 16) for byte in hex_data]
        # print(can_data)
        
        # Extract bytes 4 to 7 (position data)
        byte_4 = can_data[0]
        byte_5 = can_data[1]
        byte_6 = can_data[2]
        byte_7 = can_data[3]
        
        # Combine bytes 4 to 7 into a single 32-bit integer (little-endian)
        combined_value = (byte_7 << 24) | (byte_6 << 16) | (byte_5 << 8) | byte_4
        
        # Handle signed 32-bit values
        if combined_value >= 0x80000000:
            combined_value -= 0x100000000
        
        # Convert to degrees (0.01 degrees per LSB)
        position_degrees = combined_value / 100.0

        detected.motor_id = can_id - 100
        detected.motor_pos = abs(int(position_degrees))

        pub.publish(detected)
        rospy.sleep(0.01)

def monitor_terminal(command, target):
    """
    Monitors a terminal command and extracts lines containing the target value.

    :param command: The terminal command to monitor (e.g., "candump can0").
    :param target_value: The target value to search for in the command output.
    """
    # Start the terminal command process
    process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, shell=True)

    # Loop to continuously read the output from the command
    while not rospy.is_shutdown():
        # Read a line from the command output
        line = process.stdout.readline()

        # Check if the line contains the target value
        if target in line:
            parse_motor_reply(line.strip())


def main():

    monitor_terminal("candump can0", "92")

if __name__ == "__main__":
    
    main()
