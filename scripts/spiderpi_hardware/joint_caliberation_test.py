import math
from common.ros_robot_controller_sdk import Board
from utils import set_joint_values, reset_joints
import numpy as np
import time

board = Board()
# board.set_buzzer(1900, 0.1, 0.9, 1) # The buzzer sounds at the frequency of 1900 Hz for 0.1 seconds, followed by a pause of 09 seconds, and repeats this pattern once)

reset_joints(board)
input("home")

# body_leg_x
# indices = np.array([7,4,1,10,13,16], dtype=int)-1
# leg_x_1_2
indices = np.array([8,5,2,11,14,17], dtype=int)-1
# leg_x_2_3
# indices = np.array([9,6,3,12,15,18], dtype=int)-1

conf = np.zeros(18)

joint_value = np.pi/6
conf[indices] = -joint_value
set_joint_values(board, conf, 1)
input(f'target_value: {-joint_value}')

conf[indices] = joint_value
set_joint_values(board, conf, 2)
input(f'target_value: {joint_value}')

reset_joints(board)