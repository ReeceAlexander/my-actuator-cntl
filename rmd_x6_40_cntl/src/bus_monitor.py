#!/usr/bin/env python3

import rospy
import subprocess
import re
import os
import time
import math
from rmd_x6_40_cntl.msg import Feedback

# Publish to the "/motor_feedback" topic
pub = rospy.Publisher('motor_feedback', Feedback, queue_size=1)
rospy.init_node('bus_monitor')

def extract_position_from_can_message(can_message):
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

    if int(parts[1]) == 241 or int(parts[1]) == 242 or int(parts[1]) == 243 or int(parts[1]) == 244:

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

        detected = Feedback()
        detected.motor_id = int(parts[1]) - 100
        detected.motor_pos = abs(int(position_degrees))
        pub.publish(detected)
        rospy.sleep(0.01)
        
        # print(f"Motor {(int(parts[1]) - 100)} absolute position: {position_degrees:.2f} degrees")

# Example usage:
# can_message = "can0  244   [8]  92 00 00 00 A6 53 8E FF"
# position = extract_position_from_can_message(can_message)
# print(f"Motor absolute position: {position:.2f} degrees")


def parse_motor_reply(can_message):
    """
    Parses a CAN motor reply message to extract motor CAN ID, temperature, current, speed, and position.

    :param can_message: CAN message in the format 'can0  244   [8]  A2 17 E8 FF CC FF 65 02'.
    :return: A dictionary containing motor CAN ID, temperature (°C), current (A), speed (dps), and position (degrees).
    """

    # print(can_message)

    # Split the input string to extract the CAN ID and data bytes
    message_parts = can_message.split()
    #print(len(message_parts))
    
    # Extract CAN ID (e.g., '244' from 'can0  244   [8]')
    can_id = int(message_parts[1]) - 100  # Convert the CAN ID from hex to decimal
    
    if can_id != 241 or can_id != 242 or can_id != 243 or can_id != 244:

        # Extract the last 8 data bytes
        data_bytes = message_parts[3:]  
        #print(len(data_bytes))

        # Convert the data bytes from hex to decimal for processing
        data = [int(byte, 16) for byte in data_bytes]

        # Extract motor information from the data
        temperature = data[1]  # Motor temperature in degrees Celsius

        # Torque current: Combine data[2] and data[3] as a 16-bit signed integer
        current_raw = (data[2] << 8) | data[3]
        if current_raw >= 0x8000:  # Handle 16-bit signed integer
            current_raw -= 0x10000
        current = current_raw * 0.01  # Current in Amps (scaled by 0.01)

        # Motor speed: Combine data[4] and data[5] as a 16-bit signed integer
        speed_raw = (data[4] << 8) | data[5]
        if speed_raw >= 0x8000:  # Handle 16-bit signed integer
            speed_raw -= 0x10000
        speed = speed_raw  # Speed in dps (degrees per second)

        # Motor position: Combine data[6] and data[7] as a 16-bit signed integer
        position_raw = (data[6] << 8) | data[7]
        if position_raw >= 0x8000:  # Handle 16-bit signed integer
            position_raw -= 0x10000
        position = position_raw  # Position in degrees

        # Create the message template
        detected = Feedback()

        # Sort and publish parsed data
        detected.motor_id = can_id
        detected.motor_temp = temperature
        detected.motor_current = current
        detected.motor_speed = speed
        detected.motor_pos = position

        pub.publish(detected)
        rospy.sleep(0.01)

def monitor_terminal(command, target_value):
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
        if target_value in line:
            # If found, extract the entire line or other specified parts
            # print(f"Target detected: {line.strip()}")
            # parse_motor_reply(line.strip())
            extract_position_from_can_message(line.strip())

def main():

    monitor_terminal("candump can0", "92")

if __name__ == "__main__":
    
    main()
