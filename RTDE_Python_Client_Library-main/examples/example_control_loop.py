#!/usr/bin/env python

import sys
import logging
import rtde.rtde as rtde
import rtde.rtde_config as rtde_config
import robotiq_gripper
import time
import os

ROBOT_HOST = "143.167.1.210"
ROBOT_PORT = 30004
config_filename = "control_loop_configuration.xml"
GRIPPER_PORT = 63352

def log_info(gripper):
    print(f"Pos: {str(gripper.get_current_position()): >3}  "
          f"Open: {gripper.is_open(): <2}  "
          f"Closed: {gripper.is_closed(): <2}  ")

print("Creating gripper...")
gripper = robotiq_gripper.RobotiqGripper()
print("Connecting to gripper...")
gripper.connect(ROBOT_HOST, GRIPPER_PORT)
print("Activating gripper...")
gripper.activate()

print("Testing gripper...")

keep_running = True

logging.getLogger().setLevel(logging.INFO)

conf = rtde_config.ConfigFile(config_filename)
state_names, state_types = conf.get_recipe("state")
setp_names, setp_types = conf.get_recipe("setp")
watchdog_names, watchdog_types = conf.get_recipe("watchdog")

con = rtde.RTDE(ROBOT_HOST, ROBOT_PORT)
con.connect()

con.get_controller_version()

con.send_output_setup(state_names, state_types)
setp = con.send_input_setup(setp_names, setp_types)
watchdog = con.send_input_setup(watchdog_names, watchdog_types)

# Setpoints to move the robot to
waypoints = [
    [0.75844, 0.15630, 0.3, 2.456, -2.069, 0.094],
    [0.75844, 0.15630, 0.19713, 2.456, -2.069, 0.094],
    [0.75844, 0.15630, 0.3, 2.456, -2.069, 0.094],
    [-0.63467, 0.68356, 0.3, 2.471, -2.036, 0.134],
    [-0.63467, 0.68356, 0.19713, 2.471, -2.036, 0.134],
    [-0.63467, 0.68356, 0.3, 2.471, -2.036, 0.134]
]

# Initialize parameters
setp.input_double_register_0 = 0
setp.input_double_register_1 = 0
setp.input_double_register_2 = 0
setp.input_double_register_3 = 0
setp.input_double_register_4 = 0
setp.input_double_register_5 = 0
setp.speed_slider_fraction = 1.0  # Start with default speed (full speed)
watchdog.input_int_register_0 = 1
setp.speed_slider_mask = 1

# Initialize gripper state (0: open, 1: close)
gripper_state = 0
current_waypoint_index = 0

def setp_to_list(sp):
    return [sp.__dict__["input_double_register_%i" % i] for i in range(6)]

def list_to_setp(sp, list):
    for i in range(6):
        sp.__dict__["input_double_register_%i" % i] = list[i]
    return sp

def check_event_file():
    return os.path.isfile("flag.txt")

if not con.send_start():
    sys.exit()


# Function to control the gripper
def control_gripper(state):
    if state == 1:  # Close gripper
        gripper.move_and_wait_for_pos(255, 255, 255)
    else:  # Open gripper
        gripper.move_and_wait_for_pos(0, 255, 255)
    log_info(gripper)

# Track if the file was present in the last iteration
file_was_present = False
move_completed = True
count = 0
while keep_running:
    state = con.receive()

    if state is None:
        break

    # Debugging output to check the internal state
    # Debugging output to check the internal state
    if move_completed and state.output_int_register_0 == 1:
        move_completed = False

        # Determine current setpoint and gripper state
        current_setpoint = waypoints[current_waypoint_index]
        list_to_setp(setp, current_setpoint)
        print("Moving to pose = " + str(current_setpoint))
        con.send(setp)
        watchdog.input_int_register_0 = 1

        print(current_waypoint_index)
        # Determine if we need to open or close the gripper
        gripper_state = 0 if current_waypoint_index in [0,1,5] else 1

        control_gripper(gripper_state)

        # Update waypoint index
        current_waypoint_index = (current_waypoint_index + 1) % len(waypoints)

    elif not move_completed and state.output_int_register_0 == 0:
        move_completed = True
        watchdog.input_int_register_0 = 0

    con.send(watchdog)

con.send_pause()
con.disconnect()
