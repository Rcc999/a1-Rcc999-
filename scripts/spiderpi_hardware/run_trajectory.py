from common.ros_robot_controller_sdk import Board
from utils import set_joint_values, reset_joints, get_joint_index_remap
import numpy as np
import json
import os
import time

board = Board()
# board.set_buzzer(1900, 0.1, 0.9, 1) # The buzzer sounds at the frequency of 1900 Hz for 0.1 seconds, followed by a pause of 09 seconds, and repeats this pattern once)

# load trajectory json
HERE = os.path.dirname(__file__)
traj_path = os.path.join(HERE,"trajectory.json")
with open(traj_path, 'r') as f:
    data = json.load(f)

dt = data['dt']
# first 6 joints are base position and orientation
joint_names = data['joint_names'][6:]
trajectory = data['trajectory']
index_map = get_joint_index_remap(joint_names)

reset_joints(board)
# input("Home, ready to start...")

# execute trajectory
for conf_list in trajectory:
    conf = np.array(conf_list)
    set_joint_values(board, conf[6:][index_map], dt)
    time.sleep(dt)

reset_joints(board)