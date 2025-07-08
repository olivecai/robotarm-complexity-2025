'''
July 2 2025

solve for milestones from goal towards initial

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
    def __init__(self, tolerance=1e-3, practical_tolerance=1e-2, maxiter=200, alpha=1e-1, resolution = 20, show_each_convergence_region = 1):
        self.tolerance =tolerance #residual tolerance to stop iterations
        self.practical_tolerance = practical_tolerance
        self.maxiter = maxiter
        self.alpha = alpha #dampening
        self.resolution = resolution
        self.current_jacobian = None
        self.show_each_convergence_region = show_each_convergence_region

class Trajectory():
    def __init__(self, ets: rtb.ETS, joint_ranges : list, joint_ranges_full: list, convergence_algorithm_params : dict = None): 
        '''
        ets: robotics toolkit ets to perform invkin on
        convergence_algorithm_params: dict, create convergence algorithm to run plan_trajectory_backwards
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
        self.currMilestoneQ = None
        self.milestoneQ = []
        self.closeEnoughTolerance = 1e-1
        self.jointspace_points=[]
        if convergence_algorithm_params == None:
            self.algorithm = ConvergenceAlgorithm()
        else:
            self.algorithm = ConvergenceAlgorithm(**convergence_algorithm_params) #pass the dictionary as params
        self.planning_complete = 0 #check this every milestone. if the closest point is within some tolerance of the starting point, then planning is complete and we can see how successful our inverse kinematics is.

        # DELIVERABLES
        self.closest_success_point_to_starting_position_dist = 0
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

    def assign_trajectory_backwards(self, initialQ, desiredP):
        '''
        Assign the starting joint configuration and its corresponding starting cartesian space point, and the desired cartesian space point.
        NOTE: Currently the desired point will be in 3D Cartesian Space. If we want to do VS, make a seperate function.
        '''
        self.initQ = initialQ
        self.currQ = initialQ
        self.initP = fkin(initialQ.copy(), self.ets)
        self.desiredP = desiredP
        self.currMilestoneP = desiredP #BACKWARDS

    def assign_trajectory_forwards(self, initialQ, desiredP):
        '''
        Assign the starting joint configuration and its corresponding starting cartesian space point, and the desired cartesian space point.
        NOTE: Currently the desired point will be in 3D Cartesian Space. If we want to do VS, make a seperate function.
        '''
        self.initQ = initialQ
        self.currQ = initialQ
        self.initP = fkin(initialQ.copy(), self.ets)
        self.desiredP = desiredP
        self.currMilestoneP = self.initP #FORWARDS
        self.currMilestoneQ = initialQ

    def plot_robot_trajectory(self):
        '''
        Use the Robotics Tool Kit plot robot feature to see the cartesian trajectory the robot takes, along with the joint trajectory plot in joint space.
        '''
        #print(np.array(self.trajectory))
        ret = self.ets.plot(np.array(self.trajectory), block=True)
        return
        for traj in self.trajectory:
            ret = self.ets.plot(np.array(traj), block=True)
            print(ret)
            #ax = fig.add_subplot(111, projection='3d')
            #ax.set_xlim(-2, 2)
            #ax.set_ylim(-2, 2)
            #ax.set_zlim(-2, 2)
            #plt.pause(2)
            #plt.close()
        #slider_robot_trajectory(np.array(trajectory))

    def plot_robot_milestone_joint_configs(self):
        print("Plot milestones")
        for Q in self.milestoneQ:
            print(Q)
            self.ets.plot(np.array(Q), block=True)
        
def within_joint_limits(q, joint_ranges):
    return all(low <= q[i] <= high for i, (low, high) in enumerate(joint_ranges))

def invkin(Trajectory: Trajectory, startQ=None, desP=None):
    '''
    Compute the inverse kinematics for a constant Jacobian.
    Return SUCCESS bool, number of iterations, the resulting position, and the resulting joint configuration.
    '''
    Algorithm = Trajectory.algorithm
    if Trajectory.planning_complete:
        tolerance=Algorithm.practical_tolerance
    else:
        tolerance = Algorithm.tolerance

    currQ = startQ.copy() if startQ is not None else Trajectory.currQ.copy()
    desP = desP.copy() if desP is not None else Trajectory.currMilestoneP.copy()

    J = centraldiff_jacobian(currQ, Trajectory.ets)


    for i in range(Algorithm.maxiter):
        currP = fkin(currQ.copy(), Trajectory.ets)
        if Trajectory.planning_complete:
            print("currQ:",currQ,"currP:", currP)
        errorP = desP - currP
        error = np.linalg.norm(errorP)

        out_of_bounds=0
        if not within_joint_limits(currQ, Trajectory.joint_ranges_full):
            ret=  0, i, currP, currQ
            out_of_bounds=1
        if out_of_bounds:
            print("Out of bounds!")
            break
  
        if Trajectory.planning_complete==1:
            Trajectory.trajectory.append(currQ.copy())
            Trajectory.iterations +=1

        ret=None

        if error < tolerance:
            ret = 1, i, currP, currQ
            break

        corrQ = np.linalg.pinv(J) @ errorP

        currQ += Algorithm.alpha * corrQ

    if ret is None:
        ret=0, i, currP, currQ

    if Trajectory.planning_complete==1:
        Trajectory.currQ = currQ
    
    return ret

def arrays_are_similar(arr1, arr2, tol):
    '''
    check if two numpy arrays are similar to one another.
    '''
    #print("arr1-arr2:",np.abs(arr1.copy()-arr2.copy() ))
    diff = np.abs(arr1.copy()-arr2.copy())
    less_than_mask = diff < tol
    all_less = np.all(less_than_mask.copy())
    return all_less #if the difference is all under tolerance, then the arrays must be very similar.

def get_convergence_region_joint_cart_space_backwards(Trajectory: Trajectory):
    '''
    CARTESIAN SPACE
    iterate over the joint space to get the convergence region for a desired POINT. 
    modifies object Trajectory.
    visualizes the convergence plots if show_each_convergence_region == 1 in the object Algorithm.
    returns None
    '''
    Algorithm : ConvergenceAlgorithm = Trajectory.algorithm

    num_samples = 888  # Tune this for accuracy vs speed

    random_joint_configs = [
        np.array([np.random.uniform(low, high) for (low, high) in Trajectory.joint_ranges])
        for _ in range(num_samples)
    ]

    # parallel list of cartesian space and values.
    robot_space = [] # joint space fkin into cartesian + jointspace
    robot_values = [] # value of each cartesian + joint position
    
    #have each of these values be a different shape
    success_values= [] #star
    fail_values= [] #square
    near_initial_values= [None] * num_samples #circle
    near_curr_milestone_values= [None] * num_samples #diamond

    sum_error = 0
    greatest_distance_in_convergence_region = 0 #get the largest distance
    greatest_distance_in_convergence_region_point = None
    closest_success_point_to_starting_position_distance = np.inf #get the smallest distance
    closest_success_point_to_starting_position = None #get the actual cartesian point

    robot_space_diff = np.inf

    init_robot_vector = np.concatenate((Trajectory.initP, Trajectory.initQ))

    for Q in random_joint_configs:
        robot_pos = fkin(Q, Trajectory.ets) #it could potentially be better to look at the first joint instead of all joints, but let's see first. maybe not.
        #robot vector: the joint space part of the vector should be measured before each sample is put through inverse kinematics.
        robot_vector = np.concatenate((robot_pos.copy(), Q.copy()))
        robot_space.append(robot_vector)
        isuccess, iiters, iP, iQ = invkin(Trajectory, Q)

        ierror = np.linalg.norm(Trajectory.currMilestoneP - iP) #NOTE: error IS the distance magnitude

        if isuccess:
            difference = np.linalg.norm(robot_vector - init_robot_vector) #try to get the vector with the least difference in joint
            if difference < robot_space_diff:
                robot_space_diff = difference
                closest_success_point_to_starting_position_distance = np.linalg.norm(robot_pos - Trajectory.initP)
                closest_success_point_to_starting_position = robot_pos #even though we consider the joint space, we still want our goal to be the cartesian position.
                closest_success_point_to_starting_position_Q = iQ

        if isuccess:
            success_values.append(1)
            fail_values.append(None)
        else:
            success_values.append(None)
            fail_values.append(1)
            
        # JUST FOR PLOTTING
        if arrays_are_similar(init_robot_vector, robot_vector, Trajectory.closeEnoughTolerance):
            print("this iteration beginning point is similar to the initial point")
            isuccess = -2
        if arrays_are_similar(Trajectory.currMilestoneP.copy() , robot_pos, Trajectory.closeEnoughTolerance):
            print("current milestone is near this iteration position, ignore joint space")
            isuccess = 2

        sum_error += ierror

        robot_values.append(isuccess)

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
        robot_space = np.array(robot_space); 
        robot_values = np.array(robot_values)
        success_values = np.array(success_values)
        fail_values = np.array(fail_values)

        fig = plt.figure(figsize=(8,6))
        ax = fig.add_subplot(111, projection='3d') #

        zeros = np.zeros(num_samples)
        
        scatter = ax.scatter(
            robot_space[:, 3],  # X
            robot_space[:, 4],  # Y
            robot_space[:, 5],  # Z #
            c=robot_values,  # Color
            marker='*',
            cmap='plasma',
            s=20)

        fig.colorbar(scatter, ax=ax, label='0:FAIL, .5: <=2 iterations, 1:SUCCESS')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z') #
        ax.set_title('Convergence in Cartesian Space')
        plt.tight_layout()
        plt.show()

    #remember we are dealing with discrete math here! therefore if closest_success_point_to_starting_position is close enough to the actual starting position, do not get another jacobian.
    if arrays_are_similar(closest_success_point_to_starting_position, Trajectory.initP, Trajectory.closeEnoughTolerance):
        closest_success_point_to_starting_position = Trajectory.initP #the closest success point and the starting point are very close
        Trajectory.planning_complete = 1
    
    #NOTE: this next block is for cases where we reuse the same points every time.
    #if we are at the point where we truly cannot refine any more, then we get stuck because the tolerance isn't low enough and the same point is the closest point every time.
    if Trajectory.closest_success_point_to_starting_position_dist == closest_success_point_to_starting_position_distance:
        Trajectory.planning_complete = 1
        return #no need to update the jacobians, since it's the same position.
    Trajectory.closest_success_point_to_starting_position_dist = closest_success_point_to_starting_position_distance

    Trajectory.jacobian_updates += 1
    Trajectory.cartesian_milestones.append(Trajectory.currMilestoneP)
    Trajectory.milestone_info[Trajectory.jacobian_updates] = (Trajectory.currMilestoneP, closest_success_point_to_starting_position_distance, greatest_distance_in_convergence_region_point, greatest_distance_in_convergence_region)
    Trajectory.milestoneQ.append(closest_success_point_to_starting_position_Q)

    if Trajectory.planning_complete: #finished up our work here
        return 
    else:
        Trajectory.currMilestoneP = closest_success_point_to_starting_position
        return
    
def get_convergence_region_at_initial(Trajectory: Trajectory):
    '''
    start at the initial position and see how many joint configurations we are able to converge to nearby. (NOTE: we could be using the spectral radius for this if we didnt want to use newtons method, but that is very heavy.)

    If we compute FORWARDS, it is very important that our milestones are the previous initial/starting position, and the desired position never changes.
    '''
    print("Calculating convergence region...")
    print("curr milestoneP:", Trajectory.currMilestoneP, "curr milestoneQ:" ,Trajectory.currMilestoneQ)
    
    Algorithm : ConvergenceAlgorithm = Trajectory.algorithm

    num_samples = 888  # Tune this for accuracy vs speed

    random_joint_configs = [
        np.array([np.random.uniform(low, high) for (low, high) in Trajectory.joint_ranges])
        for _ in range(num_samples)
    ]

    # parallel list of cartesian space and values.
    robot_space = [] # joint space fkin into cartesian + jointspace
    robot_values = [] # value of each cartesian + joint position
    
    #have each of these values be a different shape
    success_values= [] #star
    fail_values= [] #square
    near_initial_values= [None] * num_samples #circle
    near_curr_milestone_values= [None] * num_samples #diamond

    sum_error = 0
    greatest_distance_in_convergence_region = 0 #get the largest distance
    greatest_distance_in_convergence_region_point = None
    closest_success_to_goal_dist = np.inf #get the smallest distance
    closest_success_to_goal = None

    robot_space_diff = np.inf

    init_robot_vector = np.concatenate((Trajectory.initP, Trajectory.initQ)) #initial point as a vector in task and joint space at the same time!!

    for Q in random_joint_configs:
        robot_pos = fkin(Q.copy(), Trajectory.ets) #it could potentially be better to look at the first joint instead of all joints, but let's see first. maybe not.
        #robot vector: the joint space part of the vector should be measured before each sample is put through inverse kinematics.
        robot_vector = np.concatenate((robot_pos.copy(), Q.copy()))
        robot_space.append(robot_vector)
        isuccess, iiters, iP, iQ = invkin(Trajectory, startQ=Trajectory.initQ, desP=robot_pos) #start pos is the initial Q, and desired point is the fkin of current iteration's Q (aka where can my startimg joint config iterate to?)
        #note: all the "i____" values are taken after the invkin has been completed, so we get the final position, the final Q, etc from these values

        ierror_iP2dP = np.linalg.norm(iP - Trajectory.desiredP) #NOTE: error IS the distance magnitude #at the beginning of this loop the current milestone is the desired position
        ierror_currmilestoneQ2Q = np.linalg.norm(Trajectory.currMilestoneQ - Q)

        if isuccess: #then our start position successfully made it to the iteration Q position.
            #we need to access the GOAL robot vector and decide in advance which solution is better suited to our initial position. HOW?
            #does it matter which solution we take to get to the goal? well, for now, NO, it doesnt. So, why dont we just see which Q is closer to initQ? iQ doesn't matter.
            #we want: Q to be close to initQ, but iP close to desiredP. 
            #get a collection of points where the error is least. check the 
            if ierror_iP2dP < closest_success_to_goal_dist:
                closest_success_to_goal_dist = ierror_iP2dP

        
        robot_values.append(isuccess)

    
    if Algorithm.show_each_convergence_region: #default Algorithm object plots
        robot_space = np.array(robot_space); 
        robot_values = np.array(robot_values)
        success_values = np.array(success_values)
        fail_values = np.array(fail_values)

        fig = plt.figure(figsize=(8,6))
        ax = fig.add_subplot(111, projection='3d')

        zeros = np.zeros(num_samples)
        
        scatter = ax.scatter(
            robot_space[:, 3],  # X
            robot_space[:, 4],  # Y
            robot_space[:, 5],  # Z
            c=robot_values,  # Color
            marker='*',
            cmap='plasma',
            s=20)

        fig.colorbar(scatter, ax=ax, label='0:FAIL, .5: <=2 iterations, 1:SUCCESS')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title('Convergence in Cartesian Space')
        plt.tight_layout()
        plt.show()
    

def get_convergence_region_forwards(Trajectory: Trajectory):
    '''
    from initial, build up to get the final.
    
    '''

def get_convergence_region_backwards(Trajectory: Trajectory):
    '''
    CARTESIAN SPACE
    iterate over the joint space to get the convergence region for a desired POINT. 
    modifies object Trajectory.
    visualizes the convergence plots if show_each_convergence_region == 1 in the object Algorithm.
    returns None
    '''
    Algorithm : ConvergenceAlgorithm = Trajectory.algorithm

    num_samples = 888  # Tune this for accuracy vs speed

    random_joint_configs = [
        np.array([np.random.uniform(low, high) for (low, high) in Trajectory.joint_ranges])
        for _ in range(num_samples)
    ]

    # parallel list of cartesian space and values.
    cartesian_space = [] # joint space fkin into cartesian space
    cartesian_values = [] # value of each cartesian position
    
    #have each of these values be a different shape
    success_values= [] #star
    fail_values= [] #square
    near_initial_values= [None] * num_samples #circle
    near_curr_milestone_values= [None] * num_samples #diamond

    sum_error = 0
    greatest_distance_in_convergence_region = 0 #get the largest distance
    greatest_distance_in_convergence_region_point = None
    closest_success_point_to_starting_position_distance = np.inf #get the smallest distance
    closest_success_point_to_starting_position = None #get the actual cartesian point

    for Q in random_joint_configs:
        print("Q:", Q)
        print("Q[0]", Q[0])
        cartesian_pos = fkin(Q, Trajectory.ets)
        cartesian_space.append(cartesian_pos)
        isuccess, iiters, iP, iQ = invkin(Trajectory, Q)

        print(iP, Trajectory.currMilestoneP, Trajectory.initP, Trajectory.desiredP)

        ierror = np.linalg.norm(Trajectory.currMilestoneP - iP) #NOTE: error IS the distance magnitude

        if isuccess:
            success_values.append(1)
            fail_values.append(None)
        else:
            success_values.append(None)
            fail_values.append(1)
            
        if isuccess: #iff this point successfully converged, calculate TWO THINGS: the farthest success point, and the closest success point to the initial position.
            #'''
            if ierror > greatest_distance_in_convergence_region:
                greatest_distance_in_convergence_region_point = iP.copy()
                greatest_distance_in_convergence_region = ierror#'''

            dist_to_initP = np.linalg.norm(cartesian_pos - Trajectory.initP)
            if dist_to_initP < closest_success_point_to_starting_position_distance:
                isuccess = -1
                closest_success_point_to_starting_position = cartesian_pos
                closest_success_point_to_starting_position_Q = Q
                closest_success_point_to_starting_position_distance = dist_to_initP
                
            #these next few lines are very optional, have no impact on the code logic, and serve to visualize the plots.
            #if iiters <=2: 
            #    isuccess = 0.5 # the only purpose this serves is to differentiate the color in the plot to see where the catresian goal actually is

        # JUST FOR PLOTTING
        if arrays_are_similar(Trajectory.initP.copy(), cartesian_pos, Trajectory.closeEnoughTolerance):
            print("this iteration beginning point is similar to the initial point")
            isuccess = -2
        if arrays_are_similar(Trajectory.currMilestoneP.copy() , cartesian_pos, Trajectory.closeEnoughTolerance):
            print("current milestone is near this iteration position")
            isuccess = 2


        sum_error += ierror

        cartesian_values.append(isuccess)

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
        cartesian_space = np.array(cartesian_space); 
        cartesian_values = np.array(cartesian_values)
        success_values = np.array(success_values)
        fail_values = np.array(fail_values)

        fig = plt.figure(figsize=(8,6))
        ax = fig.add_subplot(111, projection='3d')
        
        scatter = ax.scatter(
            cartesian_space[:, 0],  # X
            cartesian_space[:, 1],  # Y
            cartesian_space[:, 2],  # Z
            c=cartesian_values,  # Color
            marker='*',
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
    if arrays_are_similar(closest_success_point_to_starting_position, Trajectory.initP, Trajectory.closeEnoughTolerance):
        closest_success_point_to_starting_position = Trajectory.initP #the closest success point and the starting point are very close
        Trajectory.planning_complete = 1
    
    #NOTE: this next block is for cases where we reuse the same points every time.
    #if we are at the point where we truly cannot refine any more, then we get stuck because the tolerance isn't low enough and the same point is the closest point every time.
    if Trajectory.closest_success_point_to_starting_position_dist == closest_success_point_to_starting_position_distance:
        Trajectory.planning_complete = 1
        return #no need to update the jacobians, since it's the same position.
    Trajectory.closest_success_point_to_starting_position_dist = closest_success_point_to_starting_position_distance

    Trajectory.jacobian_updates += 1
    Trajectory.cartesian_milestones.append(Trajectory.currMilestoneP)
    Trajectory.milestone_info[Trajectory.jacobian_updates] = (Trajectory.currMilestoneP, closest_success_point_to_starting_position_distance, greatest_distance_in_convergence_region_point, greatest_distance_in_convergence_region)
    Trajectory.milestoneQ.append(closest_success_point_to_starting_position_Q)
    
    if Trajectory.planning_complete: #finished up our work here
        return 
    else:
        Trajectory.currMilestoneP = closest_success_point_to_starting_position
        return


def plan_trajectory_backwards(Trajectory: Trajectory):
    Algorithm : ConvergenceAlgorithm = Trajectory.algorithm

    attempts=0
    while Trajectory.planning_complete==0 and attempts<10:
        attempts+=1
        #get_convergence_region_backwards(Trajectory)
        get_convergence_region_joint_cart_space_backwards(Trajectory)
    Trajectory.planning_complete=1
    
    # after we run get_convergence_region_backwards, our Trajectory object has been populated and is ready to perform.
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
        print("Currently on the Jacobian update i=", i)
        Trajectory.currMilestoneP = Trajectory.cartesian_milestones[Trajectory.jacobian_updates-i-1] #work backwards! FILO
        invkin(Trajectory)
    
    print("Complete!")

    print("Iterations taken to converge:")
    print(Trajectory.iterations)

def plan_trajectory_forwards(Trajectory: Trajectory):
    Algorithm : ConvergenceAlgorithm = Trajectory.algorithm

    attempts=0
    while Trajectory.planning_complete==0 and attempts<10:
        attempts+=1
        get_convergence_region_at_initial(Trajectory)
    Trajectory.planning_complete=1
    
    # after we run get_convergence_region_backwards, our Trajectory object has been populated and is ready to perform.
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
        print("Currently on the Jacobian update i=", i)
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
    joint_limits2dof_full = [(-np.pi, np.pi), (-np.pi, 2*np.pi)]
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

    ets, joint_limits, joint_limits_full  = dofdylan
    ########################################################################

    initQ = np.array([0.,  0.0, 0.0]) #1.23197905 0.07786037 1.07792537 #1.47876316  0.65113557 -0.98678046
    desiredP= np.array([0.0,0.85,0.0])

    traj1 = Trajectory(ets, joint_limits, joint_limits_full)
    #traj1.assign_trajectory_forwards(initQ, desiredP)
    traj1.assign_trajectory_backwards(initQ,desiredP)

    if 0:
        plan_trajectory_backwards(traj1)

        ets.plot(initQ, block=True)
        traj1.plot_robot_trajectory()
        traj1.plot_robot_milestone_joint_configs()
        ets.plot(initQ, block=True)
    else:
        traj1.planning_complete=1
        success, i, currP, currQ = invkin(traj1)
        ets.plot(initQ, block=False)
        traj1.plot_robot_trajectory()
        if success:
            print("SUCCESS, iterations:", i)
        else:
            print("FAILURE, currP:", currP, "currQ:", currQ)
    

main()