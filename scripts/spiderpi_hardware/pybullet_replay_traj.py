import pybullet_planning as pp
import pybullet as p
import json
import time

pp.connect(use_gui=True)
robot_urdf = r"data\robots\spiderpi\spiderpi.urdf"
robot = pp.load_pybullet(robot_urdf, fixed_base=False, cylinder=False)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1, physicsClientId=pp.CLIENT)

# load trajectory json
traj_path = r"data\trajectory.json"
with open(traj_path, 'r') as f:
    data = json.load(f)

dt = data['dt']
# first 6 joints are base position and orientation
joint_names = data['joint_names'][6:]
trajectory = data['trajectory']
joints = pp.joints_from_names(robot, joint_names)

# execute trajectory
for conf in trajectory:
    base_pose = pp.Pose(point=pp.Point(x=conf[2], y=conf[0], z=conf[1]))
    pp.set_pose(robot, base_pose)
    pp.set_joint_positions(robot, joints, conf[6:])
    time.sleep(dt)

pp.wait_if_gui()
pp.disconnect()
