'''
Curtis Johnson July 2024

Developed for the Baloo hardware paper to pick up a variety of objects. 

This is a simple open-loop pressure trajectory controller hand-designed to hug each object. 

This was designed for the right arm to be at 45 degrees and the left arm to be at 67.5 degrees, 
to be offset from each other and avoid self collisions.
'''

import numpy as np
import rospy
from baloo_hardware.baloo_hw_interface import BalooHWInterface

rospy.init_node("pressure_hugger")

baloo = BalooHWInterface()

input("Press Enter to home elevator...")

homed, message = baloo.elevator.home_elevator()

if not homed:
    raise Exception(message)
else:
    print("Elevator homed")

state = "START AT TOP"
rospy.loginfo(f"State: {state}")
object_placed = False

#change these to accomodate all of the objects in front of Baloo and not hit the floor.
N = 100
left_lift_j0_pressure_traj = np.linspace(np.zeros(4),
                                         np.array([170, 0, 90, 90]), N)
left_grab_j0_pressure_traj = np.linspace(np.array([170, 0, 90, 90]),
                                         np.array([90, 90, 300, 0]), N)

left_letgo_j0_pressure_traj = np.linspace(np.array([90, 90, 300, 0]),
                                          np.array([170, 0, 90, 90]), N)

left_lift_j1_pressure_traj = np.linspace(np.zeros(4), np.array([90, 0, 90, 0]),
                                         N)
left_grab_j1_pressure_traj = np.linspace(np.array([90, 0, 90, 0]),
                                         np.array([0, 200, 300, 0]), N)
left_letgo_j1_pressure_traj = np.linspace(np.array([0, 200, 300, 0]),
                                          np.array([90, 0, 90, 0]), N)

right_lift_j0_pressure_traj = np.linspace(np.zeros(4),
                                          np.array([90, 90, 200, 0]), N)

right_grab_j0_pressure_traj = np.linspace(np.array([90, 90, 200, 0]),
                                          np.array([300, 0, 100, 200]), N)

right_letgo_j0_pressure_traj = np.linspace(np.array([300, 0, 100, 200]),
                                           np.array([90, 90, 200, 0]), N)

right_lift_j1_pressure_traj = np.linspace(np.zeros(4), np.array([90, 0, 90,
                                                                 0]), N)

right_grab_j1_pressure_traj = np.linspace(np.array([90, 0, 90, 0]),
                                          np.array([90, 0, 90, 300]), N)

right_letgo_j1_pressure_traj = np.linspace(np.array([90, 0, 90, 300]),
                                           np.array([90, 0, 90, 0]), N)

#vent arms if they are pressurized
baloo.left_arm.send_pressure_commands([np.zeros(4), np.zeros(4), np.zeros(4)])
baloo.right_arm.send_pressure_commands([np.zeros(4), np.zeros(4), np.zeros(4)])

height_cmd = 0

while not rospy.is_shutdown():

    if state == "START AT TOP":
        baloo.elevator.send_height_command(0)

        if np.isclose(baloo.elevator.get_height(), 0, atol=.001):
            state = "LIFT ARMS"
            rospy.loginfo(f"State: {state}")

    #move arms up to initial position to not collide with floor
    elif state == "LIFT ARMS":
        for i in range(N):
            left_j0_pressures = left_lift_j0_pressure_traj[i, :]
            right_j0_pressures = right_lift_j0_pressure_traj[i, :]
            left_j1_pressures = left_lift_j1_pressure_traj[i, :]
            right_j1_pressures = right_lift_j1_pressure_traj[i, :]

            baloo.left_arm.send_pressure_commands(
                [left_j0_pressures, left_j1_pressures,
                 np.zeros(4)])
            baloo.right_arm.send_pressure_commands(
                [right_j0_pressures, right_j1_pressures,
                 np.zeros(4)])

            rospy.sleep(.05)

        state = "LOWER"
        rospy.loginfo(f"State: {state}")

    elif state == "LOWER":
        baloo.elevator.send_height_command(height_cmd)

        #if within 1mm of target, move to next state
        if np.isclose(baloo.elevator.get_height(), height_cmd, atol=.001):
            state = "HUG"
            rospy.loginfo(f"State: {state}")

    elif state == "HUG":
        if not object_placed:
            input(
                "Press Enter after the object is placed in front of Baloo...")
            object_placed = True

        for i in range(N):
            left_j0_pressures = left_grab_j0_pressure_traj[i, :]
            right_j0_pressures = right_grab_j0_pressure_traj[i, :]
            left_j1_pressures = left_grab_j1_pressure_traj[i, :]
            right_j1_pressures = right_grab_j1_pressure_traj[i, :]

            baloo.left_arm.send_pressure_commands(
                [left_j0_pressures, left_j1_pressures,
                 np.zeros(4)])

            baloo.right_arm.send_pressure_commands(
                [right_j0_pressures, right_j1_pressures,
                 np.zeros(4)])

            rospy.sleep(.05)

        state = "RAISE"
        rospy.loginfo(f"State: {state}")
        rospy.sleep(3)  #wait for pressures to settle

    elif state == "RAISE":
        baloo.elevator.send_height_command(0)

        if np.isclose(baloo.elevator.get_height(), -0.0, atol=.001):
            state = "DROP"
            rospy.loginfo(f"State: {state}")

            input("Press Enter to lower the object...")

    elif state == "DROP":
        #go back down to drop object on the floor
        baloo.elevator.send_height_command(height_cmd)

        if np.isclose(baloo.elevator.get_height(), height_cmd, atol=.001):
            state = "LET GO"
            rospy.loginfo(f"State: {state}")

    elif state == "LET GO":

        for i in range(N):
            left_j0_pressures = left_letgo_j0_pressure_traj[i, :]
            right_j0_pressures = right_letgo_j0_pressure_traj[i, :]
            left_j1_pressures = left_letgo_j1_pressure_traj[i, :]
            right_j1_pressures = right_letgo_j1_pressure_traj[i, :]

            baloo.left_arm.send_pressure_commands(
                [left_j0_pressures, left_j1_pressures,
                 np.zeros(4)])

            baloo.right_arm.send_pressure_commands(
                [right_j0_pressures, right_j1_pressures,
                 np.zeros(4)])

            rospy.sleep(.05)

        state = "GO HOME"
        rospy.loginfo(f"State: {state}")

    elif state == "GO HOME":
        baloo.elevator.send_height_command(0)

        if np.isclose(baloo.elevator.get_height(), 0, atol=.001):
            state = "VENT"
            rospy.loginfo(f"State: {state}")

    elif state == "VENT":
        baloo.left_arm.send_pressure_commands(
            [np.zeros(4), np.zeros(4), np.zeros(4)])

        baloo.right_arm.send_pressure_commands(
            [np.zeros(4), np.zeros(4), np.zeros(4)])

        rospy.signal_shutdown("Finished")
