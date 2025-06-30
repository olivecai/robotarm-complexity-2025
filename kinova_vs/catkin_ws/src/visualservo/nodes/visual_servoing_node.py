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

from visualservo.msg import image_point, points_array, vs_info

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

        self.joints = np.deg2rad(np.array([-0.1336059570312672, -28.57940673828129, -179.4915313720703, -120.7, 0.06742369383573531, -57.420898437500036, 89.88030242919922]))
        self.kinova.send_joint_angles(np.copy(self.joints))

        self.correction_joints = None
        self.systems_of_equations = None #it's easier to just stack the systems of equations together 
        self.error_vector = None # des - curr. #note that this error vector will change nonstop, so cache in local function if need it constant for an iteration.
        self.residual = None #how far away from goal
        self.curr_cartesian = None
        self.jacobian = None
        self.inv_jacobian = None

        self.stop_pub = rospy.Publisher(f"visualservoing/stop_bool", Bool, queue_size=1)
        self.vs_info_pub = rospy.Publisher(f"visualservoing/vs_info", vs_info, queue_size=1)
        self.stop_pub.publish(0)

        #for each camera in camera index list, set up a SUBSCRIBER to get the points, 
        # wait for points publishers...
        rospy.Subscriber("/visualservoing/errors", points_array, self.update_errors)
        rospy.Subscriber("/visualservoing/current_position", points_array, self.update_cur_pos)
        # when the points publisher starts publishing, then compute a MoveRobot :
        #self.vis_servo()

    def update_errors(self, msg):
        #rospy.loginfo(f"errors: {msg.data}")
        self.error_vector = np.array(msg.data)
        #rospy.loginfo(f"error vector: {self.error_vector}")

    def update_cur_pos(self, msg):
        #rospy.loginfo(f"cur_pos: {msg.data}")
        self.curr_cartesian = np.array(msg.data)

    def get_move(self):
        '''
        query self attributes {inverse_jacobian, error_vector, damping_value}
        and calculate correction_joints
        '''
        return -self.damping_value * (self.inv_jacobian @ self.error_vector.T).T

    def vis_servo(self):
        self.stop_pub.publish(0)
        rospy.loginfo("Visual Servoing beginning...")

        # TODO: for now, only compute the central differences at the beginning.
        self.calc_jacobian()

        for i in range(self.max_iterations):
            rospy.loginfo(f"iteration {i} of visual servoing...")
            self.residual = np.linalg.norm(self.error_vector)
            if self.residual <= self.res_tol:
                self.stop_pub.publish(1)
                vs_success = vs_info()
                vs_success.success=1
                vs_success.iterations = i
                vs_success.error_vector = self.error_vector
                self.vs_info_pub.publish(vs_success)
                rospy.loginfo("Visual Servoing successfully converged. Shutting down other nodes...")
                return

            self.correction_joints = self.get_move()
            rospy.loginfo(f"change in joints: {self.correction_joints}")

            self.move(self.correction_joints)
            rospy.sleep(0.5)
        
        vs_fail = vs_info()
        vs_fail.success=1
        vs_fail.iterations = i
        vs_fail.error_vector = self.error_vector
        self.vs_info_pub.publish(vs_fail)
        rospy.loginfo("Visual Servoing failed to converge. Shutting down other nodes...")
        return

        
    def move(self, delta):
        '''
        move 4-dof arm
        '''
        self.joints[0] += delta[0]
        self.joints[1] += delta[1]
        self.joints[3] += delta[2]
        self.joints[5] += delta[3]
        
        rospy.loginfo(f"Sending Joints: {self.joints}")
        self.kinova.send_joint_angles(np.copy(self.joints))
        rospy.loginfo("Joint Move Complete.")

    def calc_jacobian(self):
        rospy.loginfo("Computing Central Differences Jacobian...")
        #perturb each joint individually and see the corresponding cartesian projection change.
        self.jacobian = np.zeros((self.error_vector.size, self.num_joints))
        
        delta=0.025

        move = np.zeros(self.num_joints) #if num_joints=3, then move = [[0. 0. 0. 0.]] at this point

        for i in range(self.num_joints):
            rospy.loginfo(f"Calculating volumn {i} of Jacobian...")

            initial_error = self.error_vector #cached

            # move robot
            move[i] = delta
            self.move(move)
            rospy.sleep(0.5) 
            forward_error = self.error_vector #concurrently updated and is now cached

            # move robot
            move[i] = -2*delta
            self.move(move)
            rospy.sleep(0.5)
            backward_error = self.error_vector #cached

            rospy.loginfo(f"ERROR BEFORE: {initial_error}")
            rospy.loginfo(f"ERROR FORWARD: {forward_error}")
            rospy.loginfo(f"ERROR BACKWARD: {backward_error}")
            
            # move back to initial position.
            move[i] = delta
            self.move(move)
            
            self.jacobian[:, i] = (forward_error - backward_error) / (2*delta) #central differences 
            
            move = np.zeros(self.num_joints) #reset
        
        rospy.loginfo(f"JACOBIAN: {self.jacobian}")

        self.inv_jacobian = np.linalg.pinv(self.jacobian)

        rospy.loginfo(f"INVERSE JACOBIAN: {self.inv_jacobian}")


def main():
    rospy.sleep(5) 
    rospy.loginfo("Starting TrackingNode ...")
    node_params = dict(damping_value = 0.1, max_iterations = 30, num_joints = 4, res_tol=1e-2)
    node = VisServNode(**node_params)

    rospy.sleep(5)
    node.vis_servo()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down TrackingNode...")
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

    
        