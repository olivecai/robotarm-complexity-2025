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
    def __init__(self, tolerance=1e-3, maxiter=200, alpha=1, resolution = 15, show_each_convergence_region = 0):
        self.tolerance =tolerance #residual tolerance to stop iterations
        self.maxiter = maxiter
        self.alpha = alpha #dampening
        self.resolution = resolution
        self.current_jacobian = None
        self.show_each_convergence_region = show_each_convergence_region

class Trajectory():
    def __init__(self, ets: rtb.ETS, joint_ranges : list, joint_ranges_full: list, convergence_algorithm_params : dict =None): 
        '''
        ets: robotics toolkit ets to perform invkin on
        convergence_algorithm_params: dict, create convergence algorithm to run plan_trajectory
        '''

        # HYPERPARAMETERS 
        self.robot = rtb.Robot(ets)
        self.ets = ets
        self.joint_ranges = joint_ranges
        self.joint_ranges_full= joint_ranges_full
        self.initQ = None
        self.initP = None #important for comparing the radius of each convergence region
        self.desiredP = None
        self.currMilestoneP = None
        if convergence_algorithm_params == None:
            self.algorithm = ConvergenceAlgorithm()
        else:
            self.algorithm = ConvergenceAlgorithm(**convergence_algorithm_params) #pass the dictionary as params
        self.planning_complete = 0 #check this every milestone. if the closest point is within some tolerance of the starting point, then planning is complete and we can see how successful our inverse kinematics is.

        # DELIVERABLES
        self.iterations : int = 0 # gather this data after the trajectory has been planned. probably is highly dependent on damping.
        self.trajectory : list = [] #a list of joint configurations to plot later if needed. only update this in the invkin function
        self.jacobian_updates : int = 0 #this is the focus. this value should, after conducting trajectory, ALWAYS be >= 1. IOW if we only need the desired point's jacobian, this value should be ==1.
        self.cartesian_milestones : list = [] # let this be a FILO queue, since we plan the trajectory starting from GOAL and work up to INITIAL
        #milestone_info dictionary should have the following information:
        #   KEY: the milestone jacobian. 1==GOAL JACOBIAN, n==LAST JACOBIAN NEEDED TO CONVERGE
        #   VALUES: a tuple.
        #       - the cartesian position of the milestone (ie the final goal cartesian position is always assoc with key==1)
        #       - the distance of the current milestone to the point that is selected as the next milestone (aka, if we only need one jacobian to converge, this distance should be == the distance between the goal and the starting position.)
        #       - the furthest point in its personal region of convergence that can successfully converge to the milestone (this is not super informative but it is easy to harvest)
        
        
        #   For example, if we only need one jacobian, the desP==(0,1,0), the starting position is origin, and the furthest point that can converge to desP (0,1,0) is located at (2,2,1), the dictionary will look like:
        #       {1: ([0. 1. 0.], 1., [2. 2. 1.]) }
        #   For example, if we need two jacobians: {1: (desP, length from desP to the point Intermediate that is closest to starting point, the furthest success point to desP),
        #                                           2: (Intermediate, length from point Intermediate to the point that is closest to starting point, the furthest success point to Intermediate),
        #                                           ...}
        self.milestone_info : dict = dict() 

        # FYI: 
        #   the length of cartesian_milestones == length milestone_info == jacobian_updates
        #   the first value entry of the dictionary should be identical to the list cartesian_milestones.
        # the dictionary milestone_info uses the value of jacobian_updates as its key, so be sure to update it consistently.

    def assign_trajectory(self, initialQ, desiredP):
        '''
        Assign the starting joint configuration and its corresponding starting cartesian space point, and the desired cartesian space point.
        NOTE: Currently the desired point will be in 3D Cartesian Space. If we want to do VS, make a seperate function.
        '''
        self.initQ = initialQ
        self.initP = fkin(initialQ, self.ets)
        self.desiredP = desiredP
        self.currMilestoneP = desiredP

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
        errorP = Trajectory.currMilestoneP - currP
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

def get_convergence_region(Trajectory: Trajectory, Algorithm: ConvergenceAlgorithm):
    '''
    iterate over the joint space to get the convergence region for a desired POINT. 
    modifies object Trajectory.
    visualizes the convergence plots if show_each_convergence_region == 1 in the object Algorithm.
    returns None
    '''
    joint_space_linspace = [np.linspace(low, high, Algorithm.resolution) for (low, high) in Trajectory.joint_ranges]

    grid = np.meshgrid(*joint_space_linspace, indexing='ij')

    # stack all grids to create a dof x n x n x ... array
    joint_space_grid = np.stack(grid, axis=-1) # shape: (n, n, ..., n, dof)

    # preallocate error arrays (same shape as grid, but without dof)
    permuts_over_linspaces_shape = joint_space_grid.shape[:-1]

    # parallel list of cartesian space and values.
    cartesian_space = [] # joint space fkin into cartesian space
    cartesian_values = [] # value of each cartesian position

    sum_error = 0
    greatest_distance_in_convergence_region = 0 #get the largest distance
    closest_success_point_to_starting_position_distance = np.inf #get the smallest distance
    closest_success_point_to_starting_position = None #get the actual cartesian point
    

    for idx in np.ndindex(permuts_over_linspaces_shape):
        Q = joint_space_grid[idx].copy()
        cartesian_pos = fkin(Q, Trajectory.ets)
        cartesian_space.append(cartesian_pos)
        isuccess, iiters, iP, iQ = invkin(Trajectory, Algorithm)

        ierror = np.linalg.norm(Trajectory.currMilestoneP - iP) #NOTE: error IS the distance magnitude
        if isuccess: #iff this point successfully converged, calculate TWO THINGS: the farthest success point, and the closest success point to the initial position.
            if ierror > greatest_distance_in_convergence_region:
                greatest_distance_in_convergence_region = ierror
            
            dist_to_initP = np.linalg.norm(iP - Trajectory.initP)
            if dist_to_initP < closest_success_point_to_starting_position_distance:
                closest_success_point_to_starting_position = iP
                closest_success_point_to_starting_position_distance = dist_to_initP
            
            #these next two lines are very optional, have no impact on the code logic, and serve to visualize the plots.
            if iiters <=2: 
                isuccess = 0.5 # the only purpose this serves is to differentiate the color in the plot to see where the catresian goal actually is
    
        sum_error += ierror

        cartesian_values.append(isuccess)

    #end for

    if closest_success_point_to_starting_position_distance == np.inf:
        closest_success_point_to_starting_position_distance = None
        closest_success_point_to_starting_position = None


    if Algorithm.show_each_convergence_region: #default Algorithm object does NOT plot
        cartesian_space = np.array(cartesian_space); cartesian_values = np.array(cartesian_values)

        fig = plt.figure(figsize=(8,6))
        ax = fig.add_subplot(111, projection='3d')
        scatter= ax.scatter(cartesian_space[:, 0],  # X
                            cartesian_space[:, 1],  # Y
                            cartesian_space[:, 2],  # Z
                            c=cartesian_values,  # Color
                            cmap='plasma',
                            s=20)

        fig.colorbar(scatter, ax=ax, label='0:FAIL, .5: <=2 iterations, 1:SUCCESS')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title('Convergence in Cartesian Space')
        plt.tight_layout()
        plt.show()


    #remember we are dealing with discrete math here! therefore if closest_success_point_to_starting_position is close enough to the actual starting position, do not get another jacobian.


    Trajectory.jacobian_updates += 1
    Trajectory.currMilestoneP = closest_success_point_to_starting_position