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
    def __init__(self, tolerance=1e-1, maxiter=200, alpha=1e-1, resolution = 50, show_each_convergence_region = 1):
        self.tolerance =tolerance #residual tolerance to stop iterations
        self.maxiter = maxiter
        self.alpha = alpha #dampening
        self.resolution = resolution
        self.current_jacobian = None
        self.show_each_convergence_region = show_each_convergence_region

class Trajectory():
    def __init__(self, ets: rtb.ETS, joint_ranges : list, joint_ranges_full: list, convergence_algorithm_params : dict = None): 
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
        self.currQ = None
        self.initP = None #important for comparing the radius of each convergence region
        self.desiredP = None
        self.currMilestoneP = None
        self.closeEnoughTolerance = 1
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
        #       {1: ([0. 1. 0.], 1., [2. 2. 1.], np.linalg.norm(desP-[2. 2. 1.])) }
        #   For example, if we need two jacobians: {1: (desP, length from desP to the point Intermediate that is closest to starting point, the furthest success point to desP, that distance),
        #                                           2: (Intermediate, length from point Intermediate to the point that is closest to starting point, the furthest success point to Intermediate, that distance),
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
        self.currQ = initialQ
        self.initP = fkin(initialQ, self.ets)
        self.desiredP = desiredP
        self.currMilestoneP = desiredP

    def plot_robot_trajectory(self):
        '''
        Use the Robotics Tool Kit plot robot feature to see the cartesian trajectory the robot takes, along with the joint trajectory plot in joint space.
        '''
        print(np.array(self.trajectory))
        self.ets.plot(np.array(self.trajectory), block=False)
        plt.close('all')
        #slider_robot_trajectory(np.array(trajectory))
        
def invkin(Trajectory: Trajectory):
    '''
    Compute the inverse kinematics for a constant Jacobian.
    Return SUCCESS bool, number of iterations, the resulting position, and the resulting joint configuration.
    '''
    Algorithm = Trajectory.algorithm

    currQ = Trajectory.currQ.copy()
    J = centraldiff_jacobian(currQ.copy(), Trajectory.ets)

    for i in range(Algorithm.maxiter):
        currP = fkin(currQ.copy(), Trajectory.ets)
        errorP = Trajectory.currMilestoneP - currP
        error = np.linalg.norm(errorP)

        if Trajectory.planning_complete==1:
            Trajectory.trajectory.append(currQ.copy())
            Trajectory.iterations +=1

        ret=None

        if error < Algorithm.tolerance:
            ret = 1, i, currP, currQ
            break

        corrQ = np.linalg.pinv(J) @ errorP

        currQ += Algorithm.alpha * corrQ

    if ret is None:
        ret=0, i, currP, currQ

    if Trajectory.planning_complete==1:
        Trajectory.currQ = currQ
    
    return ret

def get_convergence_region(Trajectory: Trajectory):
    '''
    iterate over the joint space to get the convergence region for a desired POINT. 
    modifies object Trajectory.
    visualizes the convergence plots if show_each_convergence_region == 1 in the object Algorithm.
    returns None
    '''
    Algorithm : ConvergenceAlgorithm = Trajectory.algorithm

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
    greatest_distance_in_convergence_region_point = None
    closest_success_point_to_starting_position_distance = np.inf #get the smallest distance
    closest_success_point_to_starting_position = None #get the actual cartesian point
    

    for idx in np.ndindex(permuts_over_linspaces_shape):
        Q = joint_space_grid[idx].copy()
        cartesian_pos = fkin(Q, Trajectory.ets)
        cartesian_space.append(cartesian_pos)
        Trajectory.currQ = Q.copy()
        isuccess, iiters, iP, iQ = invkin(Trajectory)

        ierror = np.linalg.norm(Trajectory.currMilestoneP - iP) #NOTE: error IS the distance magnitude
        if isuccess: #iff this point successfully converged, calculate TWO THINGS: the farthest success point, and the closest success point to the initial position.
            if ierror > greatest_distance_in_convergence_region:
                greatest_distance_in_convergence_region_point = iP
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
    Trajectory.currQ = Trajectory.initQ.copy()

    #end for
    #sanity check:
    print("curr milestone:", Trajectory.currMilestoneP)
    print("curr Q:", Trajectory.currQ, "init Q:", Trajectory.initQ, "init P:", Trajectory.initP, "currMilestoneP:", Trajectory.currMilestoneP)
    print("closest success point:", closest_success_point_to_starting_position)
    print("^ distance:", closest_success_point_to_starting_position_distance)

    if closest_success_point_to_starting_position_distance == np.inf:
        closest_success_point_to_starting_position_distance = None #if this happens, that means there are absolutely no success points. 
        closest_success_point_to_starting_position = None


    if Algorithm.show_each_convergence_region: #default Algorithm object plots
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
    if np.linalg.norm(closest_success_point_to_starting_position  - Trajectory.initP) < Trajectory.closeEnoughTolerance:
        closest_success_point_to_starting_position = Trajectory.initP
        Trajectory.planning_complete = 1

    Trajectory.jacobian_updates += 1
    Trajectory.cartesian_milestones.append(Trajectory.currMilestoneP)
    Trajectory.milestone_info[Trajectory.jacobian_updates] = (Trajectory.currMilestoneP, closest_success_point_to_starting_position_distance, greatest_distance_in_convergence_region_point, greatest_distance_in_convergence_region)

    if Trajectory.planning_complete: #finished up our work here
        return 
    else:
        Trajectory.currMilestoneP = closest_success_point_to_starting_position
        return
    
def plan_trajectory(Trajectory: Trajectory):
    Algorithm : ConvergenceAlgorithm = Trajectory.algorithm

    attempts=0
    while Trajectory.planning_complete==0 and attempts<10:
        attempts+=1
        get_convergence_region(Trajectory)
    Trajectory.planning_complete=1
    
    # after we run get_convergence_region, our Trajectory object has been populated and is ready to perform.
    print("milestone info: jacobian milestone count (1==goal) : (cartesian milestone, distance from milestone to initial, farthest success point in milestone's convergence region, furthest success point in convergence region distance.)")
    print(Trajectory.milestone_info)

    line = ("###############################")
    print(line)

    print("INITIAL Q:", Trajectory.initQ, "INITIAL P:", Trajectory.initP)

    print("number of jacobian updates:", Trajectory.jacobian_updates)
    print("milestones:")
    print(Trajectory.cartesian_milestones)

    print(line)

    print("Beginning the inverse kinematics...")

    #reset Trajectory.iterations:
    Trajectory.iterations = 0 

    for i in range(Trajectory.jacobian_updates):
        Trajectory.currMilestoneP = Trajectory.cartesian_milestones[Trajectory.jacobian_updates-i-1] #work backwards! FILO
        invkin(Trajectory)
    
    print("Complete!")

    print("Iterations taken to converge:")
    print(Trajectory.iterations)


def main():
    ####################################################################
    l0=1;l1=1;l2=1;

    ets1dof = rtb.ET.Rz() * rtb.ET.tx(l0)
    joint_limits1dof = [(-np.pi/2, np.pi/2)]
    joint_limits1dof_full = [(-2*np.pi, 2*np.pi)]
    dof1 = ets1dof, joint_limits1dof, joint_limits1dof_full

    ets2dof = rtb.ET.Rz() * rtb.ET.tx(l0) * rtb.ET.Rz() * rtb.ET.tx(l1) 
    joint_limits2dof = [(0, np.pi), (-np.pi, np.pi)]  # example for 2 DOF
    joint_limits2dof_full = [(-2*np.pi, 2*np.pi), (-2*np.pi, 2*np.pi)]
    dof2 = ets2dof, joint_limits2dof, joint_limits2dof_full

    ets3dof = rtb.ET.Rz() * rtb.ET.tx(l0) * rtb.ET.Rx() * rtb.ET.tz(l1) * rtb.ET.Rx() * rtb.ET.tz(l2)
    joint_limits3dof = [(-np.pi/2, np.pi/2), (-np.pi/2, np.pi/2) , (-np.pi/2, np.pi/2)]
    joint_limits3dof_full = [(-2*np.pi/2, 2*np.pi/2), (-2*np.pi/2, 2*np.pi/2) , (-2*np.pi/2, 2*np.pi/2)]
    dof3 = ets3dof, joint_limits3dof, joint_limits3dof_full

    ets_dylan = rtb.ET.Rz() * rtb.ET.Ry(np.pi/2) * rtb.ET.Rz(np.pi) * rtb.ET.Ry() * rtb.ET.tz(0.55) * rtb.ET.Ry() * rtb.ET.tz(0.30)
    #joint_limits_dylan = [(0, np.pi/2), (0, np.pi/2) , (-np.pi/2, np.pi/2)]
    joint_limits_dylan = [(-np.pi, np.pi), (-np.pi/2, np.pi/2) , (-np.pi/2, np.pi/2)]
    joint_limits_full_dylan = [(-2*np.pi, 2*np.pi), (-2*np.pi, 2*np.pi) , (-2*np.pi, 2*np.pi)]
    dofdylan = ets_dylan, joint_limits_dylan, joint_limits_full_dylan

    ets, joint_limits, joint_limits_full  = dof2
    ########################################################################

    initQ = np.array([0,np.pi/8])
    desiredP= np.array([1.,1.,0.])

    traj1 = Trajectory(ets, joint_limits, joint_limits_full)
    traj1.assign_trajectory(initQ, desiredP)
    if 1:
        plan_trajectory(traj1)
        traj1.plot_robot_trajectory()
    else:
        traj1.planning_complete=1
        success, i, currP, currQ = invkin(traj1)
        traj1.plot_robot_trajectory()
        if success:
            print("SUCCESS, iterations:", i)
        else:
            print("FAILURE, currP:", currP, "currQ:", currQ)
    


main()