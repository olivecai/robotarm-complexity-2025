'''
July 2 2025

1. Specify CURRENT and specify GOAL
2. For GOAL, compute over the entire joint space the success/fail of convergence with a constant jacobian.
3. Get the closest point from that cloud of SUCCESS points to the GOAL POSITION
we store all the success/fail points for each point that converge to the GOAL POSITION --> then from the set of success points, we choose the closest point to the CURRENT GOAL POSITION and make that our intermediate step. REPEAT until we get to the GOAL.


'''
import numpy as np
import math
from spatialmath import SE3
from spatialmath.base import e2h, h2e
import machinevisiontoolbox as mvtb
import roboticstoolbox as rtb
import matplotlib.pyplot as plt
import plotly.express as px
import plotly.graph_objects as go
from common_robot_calculations import * 
from roboticstoolbox.models.DH import Puma560

class ConvergenceAlgorithm():
    def __init__(self, tolerance=1e-3, maxiter=200, alpha=1):
        self.tolerance =tolerance #residual tolerance to stop iterations
        self.maxiter = maxiter
        self.alpha = alpha #dampening
        self.current_jacobian = None

class Trajectory():
    def __init__(self, ets: rtb.ETS, joint_ranges : list, joint_ranges_full: list, convergence_algorithm_params : dict =None): 
        '''
        ets: robotics toolkit ets to perform invkin on
        convergence_algorithm_params: dict, create convergence algorithm to run plan_trajectory
        '''
        self.robot = rtb.Robot(ets)
        self.ets = ets
        self.joint_ranges = joint_ranges
        self.joint_ranges_full= joint_ranges_full
        self.cartesian_milestones = [] # let this be a FILO queue, since we plan the trajectory starting from GOAL and work up to INITIAL
        self.initQ = None
        self.desiredP = None
        self.jacobian_updates = 0 #this is the focus 
        self.init2newmilestone_dist_supremums = dict() #every time we update the jacobian, see how large the distance from initial to the new milestone is. This can inform us about the radius. KEYS: 1, 2, 3... Key 1 is ALWAYS associated with the desired position's jacobian. AKA if we can converge with only one jacobian, then this dictionary should be: {1: 0.00}
        self.iterations = 0
        self.trajectory = [] #a list of joint configurations to plot later if needed. only update this in the invkin function
        if convergence_algorithm_params == None:
            self.algorithm = ConvergenceAlgorithm()
        else:
            self.algorithm = ConvergenceAlgorithm(**convergence_algorithm_params) #pass the dictionary as params

    def assign_trajectory(self, initialQ, desiredP):
        '''
        Assign the starting joint configuration, and the desired cartesian space point.
        NOTE: Currently the desired point will be in 3D Cartesian Space. If we want to do VS, make a seperate function.
        '''
        self.initQ = initialQ
        self.desiredP = desiredP

    def plan_trajectory(self):
        '''
        Here is the MEAT AND POTATOES: 
        Starting from the cartesian goal position, iterate through the joint space to get the convergence region.
        Convert the joint space convergence region into the cartesian space convergence region, 
        and get the closest SUCCESS point (in cartesian space) to the cartesian point of the initial joint configuration position. 

        (Another way to think of this is: Measure the distance between the cartesian INITIAL position to every SUCCESSFUL convergence position that corresponds to the current GOAL. 
        Then, find the SUPREMUM of that set of distances: the point corresponding to this supremum is our NEW goal. AKA a milestone, push into the queue.)
        '''
        
        #for the desired point, compute the constant jacobian convergence

    def plot_robot_trajectory(self):
        '''
        Use the Robotics Tool Kit plot robot feature to see the cartesian trajectory the robot takes, along with the joint trajectory plot in joint space.
        '''
        self.ets.plot(np.array(self.trajectory), block=False)
        plt.close('all')
        #slider_robot_trajectory(np.array(trajectory))

        
def invkin(Trajectory: Trajectory, Algorithm: ConvergenceAlgorithm):
    '''
    Compute the inverse kinematics for a constant Jacobian.
    Return SUCCESS bool, number of iterations, the resulting position, and the resulting joint configuration.
    '''
    currQ = Trajectory.initQ
    for i in range(Algorithm.maxiter):
        currP = fkin(currQ, Trajectory.ets)
        errorP = Trajectory.desiredP - currP
        error = np.linalg.norm(errorP)

        Trajectory.trajectory.append(currQ.copy())

        ret=None

        if error < Algorithm.tolerance:
            ret = 1, i, currP, currQ
            break

        J = centraldiff_jacobian(currQ, Trajectory.ets)
        corrQ = np.linalg.pinv(J) @ errorP

        currQ += Algorithm.alpha * corrQ

    if not ret:
        ret=0, i, currP, currQ
    
    return ret

def calculate_error(Trajectory: Trajectory, Algorithm: ConvergenceAlgorithm):
    