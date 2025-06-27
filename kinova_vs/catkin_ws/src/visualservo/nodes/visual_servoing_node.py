#!/usr/bin/python3

'''
June 26 2025

The visual servoing node is subscribed to two topics:
1. The current p2p ERROR vector, measured by des - cur. 
2. The current cartesian position.

The functions we may need:
- calculate central differences Jacobian
- get current joint state of the robot
- tell robot to move by float32 change in radians
- when convergence error is low enough, publish stop to all nodes to gracefully kill
- 
'''
import sys
import rospy
import cv2
import cv_bridge
import numpy as np
import matplotlib.pyplot as plt

from kortex_bringup import KinovaGen3

import message_filters
from std_msgs.msg import Bool

from visualservo.msg import image_point, points_array

class VisServNode:
    def __init__(self, damping_value, max_iterations, num_joints, res_tol):
        '''
        there will exist ONE visual servoing node for each visual servoing experiment.
        '''
        rospy.init_node("visual_servo_node", anonymous=False) #we just need one visual servoing node :-)

        self.kinova : KinovaGen3 = KinovaGen3()

        self.damping_value = damping_value
        self.max_iterations = max_iterations
        self.num_joints = num_joints
        self.res_tol = res_tol

        self.joints = np.array([-0.19655300635664474, 0.8299573758083271, -2.8110765191316562, -1.0309490727538062, -0.31231795578129784, -1.1241426666756027, 1.7763640002834045])
        self.kinova.send_joint_angles(np.copy(self.joints))

        self.correction_joints = None
        self.correction_position = None
        self.systems_of_equations = None #it's easier to just stack the systems of equations together 
        self.error_vector = None # des - curr
        self.residual = None #how far away from goal
        self.curr_cartesian = None
        self.jacobian = None
        self.inv_jacobian = None

        self.stop_pub = rospy.Publisher(f"visualservoing/stop", Bool, queue_size=1)
        self.stop_pub.publish(0)

        #for each camera in camera index list, set up a SUBSCRIBER to get the points, 
        # wait for points publishers...
        rospy.Subscriber("/visualservoing/errors", points_array, self.update_errors)
        rospy.Subscriber("/visualservoing/current_position", points_array, self.update_cur_pos)
        # when the points publisher starts publishing, then compute a MoveRobot :


        # MoveRobot: we should be able to call this as a function itself AND as a callback function... measure the error and create the systems of linear equations to move the robot.
    
    def update_errors(self, msg):
        rospy.loginfo(f"errors: {msg.data}")
        self.error_vector = np.array(msg.data)

    def update_cur_pos(self, msg):
        rospy.loginfo(f"cur_pos: {msg.data}")
        self.curr_cartesian = np.array(msg.data)

    def vis_servo(self):
        if self.residual <= self.res_tol:
            self.stop_pub.publish(1)
            rospy.loginfo("Visual Servoing finished. Shutting down other nodes...")
            exit(0)

        self.stop_pub.publish(0)

        # for now, only compute the central differences at the beginning.
        
    def move(self, delta):
        """Move the wam simulation by delta

        Args:
            delta (list): delta for the 3dof
        """
        self.joints[0] += delta[0]
        self.joints[1] += delta[1]
        self.joints[3] += delta[2]
        self.joints[5] += delta[3]
        
        rospy.loginfo(f"Sending Joints: {self.joints}")
        self.kinova.send_joint_angles(np.copy(self.joints))
        rospy.loginfo("Joint Move Complete.")

    def calc_jacobian(self):
        #perturb each joint individually and see the corresponding change.
        self.jacobian = np.zeros((self.error_vector.size, self.num_joints))
        delta=0.025

        move = np.zeros(1, self.num_joints) #if num_joints=3, then move = [[0. 0. 0. 0.]] at this point








def main():
    rospy.sleep(5) 
    rospy.loginfo("Starting TrackingNode ...")
    node_params = dict(damping_value = 0.05, max_iterations = 30, num_joints = 4, res_tol=1e-2)
    node = VisServNode(**node_params)
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down TrackingNode...")
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

    
        