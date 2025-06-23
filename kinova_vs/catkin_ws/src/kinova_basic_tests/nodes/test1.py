#!/usr/bin/env python3

import numpy as np
import rospy

from kortex_bringup import KinovaGen3

#first we initialize a node
rospy.init_node('test1_node', anonymous=False) #only one node so it should not be anon

#then we create a robot node 
gen3 = KinovaGen3()

print(gen3)

angles = np.deg2rad(np.array([-0.1336059570312672, -28.57940673828129, -179.4915313720703, -147.7, 0.06742369383573531, -57.420898437500036, 89.88030242919922]))
success = gen3.send_joint_angles(angles)
print("Kinova sent home. Joints:", gen3.position)

#try moving the robot a little by specifying joint angles
angles = np.deg2rad(np.array([-0.1336059570312672, -28.57940673828129, -179.4915313720703, -147.7, 0.06742369383573531, -57.420898437500036, 30.88030242919922]))
success = gen3.send_joint_angles(angles)
print("Kinova sent to position. Joints:", gen3.position)

#then move the robot by specifying joint velocities
velocities = np.array([1,0,0,0,0,0,0])
success = gen3.send_joint_velocities(velocities)

rospy.sleep(10.)

velocities=np.array([0,0,0,0,0,0,0])
success = gen3.send_joint_velocities(velocities)
print("Kinova after joint velocity changes:", gen3.position)

gripper_float = 0.8888
success = gen3.send_gripper_command(gripper_float)
print("Kinova after gripper command:", gen3.position)

