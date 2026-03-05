# This code is adapted from Zixian Zang's code: https://github.com/XAVILLA/spiderpi_code
# Since we are using the same URDF

from common.ros_robot_controller_sdk import Board
import time
import numpy as np

DELTA_T = 0.5
# DELTA_T = 0.04
NUM_JOINTS = 18

# the motor direction and urdf axis direction is incoherent
TO_INVERT = [
    False, True, True, 
    False, True, True, 
    False, True, True, 
    True, False, False, 
    True, False, False, 
    True, False, False
]
INVERSION_ARRAY = -2*np.array(TO_INVERT, dtype=int) + 1

MOTOR_INPUT_JOINT_NAMES = [
    "body_leg_0", "leg_0_1_2", "leg_0_2_3", # 1,2,3
    "body_leg_1", "leg_1_1_2", "leg_1_2_3", # 4,5,6
    "body_leg_2", "leg_2_1_2", "leg_2_2_3", # 7,8,9
    "body_leg_5", "leg_5_1_2", "leg_5_2_3", # 10,11,12
    "body_leg_4", "leg_4_1_2", "leg_4_2_3", # 13,14,15
    "body_leg_3", "leg_3_1_2", "leg_3_2_3", # 16,17,18
]

def angle_to_pulse(angle):
    '''Returns the bus servo pulse corresponding to an angle.
       Angle has range [0, 240] corresponding to pulse range
       [0, 1000].
    
    # shoud clip to +-120
    '''
    clipped_angle = np.clip(angle, a_min=-120, a_max=120)
    return int((clipped_angle / 240) * 1000)

def pulse_to_angle(pulse):
    '''Returns the bus servo pulse corresponding to an angle.
       Angle has range [0, 240] corresponding to pulse range
       [0, 1000].
    '''
    pulse = max(0, min(pulse, 1000))
    return int(((pulse-500) / 1000) * 240)

def set_joint_values(board: Board, joint_values, duration=DELTA_T):
    # joint_values in radians, duration is in seconds!
    assert len(joint_values) == NUM_JOINTS
    joint_angles = np.degrees(joint_values)
    print(joint_angles)
    board.bus_servo_set_position(duration, [[i+1, 500 + INVERSION_ARRAY[i] * angle_to_pulse(joint_angles[i])] for i in range(NUM_JOINTS)])

def reset_joints(board: Board):
    home_values = np.zeros(18)
    set_joint_values(board, home_values, duration=2)

    # hiwonder_pose = np.array([120, 105,  75, 120, 105,  75, 120, 105,  75, 120, 135, 165, 120, 135, 165, 120, 135, 165])
    
def read_joint_angles():
    raise ValueError("Deprecated")

    for i in range(18):
        pulse = Board.getBusServoPulse(i+1)
        angle = pulse_to_angle(pulse)
        print(str(i)+"th joint:"+str(angle))