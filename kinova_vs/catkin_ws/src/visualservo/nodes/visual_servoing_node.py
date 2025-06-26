#!/usr/bin/python3

'''
June 26 2025

For each camera, create a node to publish and subscribe the image points.

have a sort of 'blocking' call that waits for the amount of points recv == amount of cameras:
when this occurs, then we can create the systems of equations to solve for the joint angles.

SUBSCRIBE to each tracking node for each set of points.

The system of equations we are trying to solve is... cam1des-cam1cur=0 and cam2des-cam2cur=0
Have a dictionary of the points to parse them, des = {}, 
For each tracking node:

'''

class VisServNode:
    def __init__(self):
        '''
        there will exist ONE visual servoing node for each visual servoing experiment.
        '''
        self.old_joints = None
        self.new_joints = None
        self.correction_joints = None
        self.correction_position = None
        self.systems_of_equations = None #it's easier to just stack the systems of equations together 
        self.error = None
        self.camera_list = 

        #for each camera in camera index list, set up a SUBSCRIBER to get the points, 
        # wait for points publishers...
        rospy.Subscriber(f"/cameras/cam{self.cam_id}/des_pnt")
        # when the points publisher starts publishing, then compute a MoveRobot :


        # MoveRobot: we should be able to call this as a function itself AND as a callback function... measure the error and create the systems of linear equations to move the robot.




    def vs_callback()
    
        