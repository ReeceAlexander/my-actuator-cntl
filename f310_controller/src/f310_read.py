#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
# from std_msgs.msg import String

import pygame
import time
import subprocess

# Initialize pygame's joystick module
pygame.init()
pygame.joystick.init()

# Check if a joystick is connected
if pygame.joystick.get_count() < 1:
    print("No joystick connected")
    pygame.quit()
    exit()

# Initialize the first joystick
joystick = pygame.joystick.Joystick(0)
joystick.init()

print(f"Joystick initialized: {joystick.get_name()}")

# Automated forward motion variable
cruise_cntl = False

# Publish to the "/cmd_vel" topic
pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
rospy.init_node('f310_input_node')

# def execute_curl_command():
#     # Command to run
#     command = ['curl', 'http://10.5.5.9/gp/gpControl/command/shutter?p=1']

#     try:
#         # Execute the command
#         result = subprocess.run(command, check=True, capture_output=True, text=True)

#         # Print the output
#         print("Command executed successfully:")
#         print(result.stdout)
    
#     except subprocess.CalledProcessError as e:
#         # Handle the error
#         print(f"Command failed with error: {e}")
#         print(f"Return code: {e.returncode}")
#         print(f"Output: {e.output}")
#         if e.stderr:
#             print(f"Error Output: {e.stderr}")

# Loop to continuously check for events
try:
    while not rospy.is_shutdown():
        pygame.event.pump()  # Process event queue

        # Buttons
        for i in range(joystick.get_numbuttons()):
            button = joystick.get_button(i)
            if button:
                
                if i == 0: # A
                    print("Photo")
                    # execute_curl_command()
                
                elif i == 1: # B
                    print("Video")

                # if i == 2: # X
                #     print("X")
                
                elif i == 3: # Y
                    print("Light")

                # if i == 4: # Left Button
                #     print("LB")
                
                # if i == 5: # Right Button
                #     print("RB")

                # if i == 6: # Back
                #     print("Back")
                
                # if i == 7: # Start
                #     print("Start")

                # if i == 8: # Logitech
                #     print("Logitech")
                
                # if i == 9: # L. Joystick
                #     print("L. Joystick")

                # if i == 10: # R. Joystick
                #     print("R. Joystick")

        crawler = Twist()

        # Axes (joysticks)
        for i in range(joystick.get_numaxes()):
            axis = joystick.get_axis(i)
            if abs(axis) > 0.1:  # Threshold to ignore small movements
                
                if i == 0 and axis < 0.0: # L. Joystick left
                    print(f"Left turn: {axis:.2f}")
                    crawler.angular.z = axis

                elif i == 0 and axis > 0.0: # L. Joystick Right
                    print(f"Right turn: {axis:.2f}")
                    crawler.angular.z = axis

                elif i == 1 and axis < 0.0 and not cruise_cntl: # L. Joystick Up
                    print(f"Forward: {axis:.2f}")
                    crawler.linear.x = abs(axis)

                elif i == 1 and axis > 0.0 and not cruise_cntl: # L. Joystick Down
                    print(f"Reverse: {axis:.2f}")
                    crawler.linear.x = axis * -1


                # if i == 2: # Left Trigger
                #     print(f"Left Trigger: {axis:.2f}")

                # if i == 3 & axis < 0: # R. Joystick left
                #     print(f"Left turn: {axis:.2f}")

                # if i == 3 & axis > 0: # R. Joystick Right
                #     print(f"Right turn: {axis:.2f}")

                # if i == 4 & axis < 0: # R. Joystick Up
                #     print(f"Forward: {axis:.2f}")

                # if i == 4 & axis > 0: # R. Joystick Down
                #     print(f"Reverse: {axis:.2f}")

                # if i == 2: # Right Trigger
                #     print(f"Right Trigger: {axis:.2f}")

        # Hats (D-pad)
        for i in range(joystick.get_numhats()):
            hat = joystick.get_hat(i)
            if hat != (0, 0):  # Only print if the D-pad is being pressed
                
                if hat == (0, 1):
                    cruise_cntl = True
                    print("CC On")
                    crawler.linear.x = 0.5
                    time.sleep(0.2)

                elif hat == (0, -1):
                    cruise_cntl = False
                    print("CC Off")
                    crawler.linear.x = 0.0
                    time.sleep(0.2)

                # if hat == (1, 1):
                #     print("NE")

                # if hat == (1, 0):
                #     print("E")

                # if hat == (1, -1):
                #     print("SE")

                # if hat == (0, -1):
                #     print("S")

                # if hat == (-1, -1):
                #     print("SW")

                # if hat == (-1, 0):
                #     print("W")

                # if hat == (-1, 1):
                #     print("NW")

        if cruise_cntl == True:
            crawler.linear.x = 0.5

        pub.publish(crawler)
        rospy.sleep(0.1)  # Sleep to reduce CPU usage

except KeyboardInterrupt:
    print("Exiting...")
finally:
    pygame.quit()
