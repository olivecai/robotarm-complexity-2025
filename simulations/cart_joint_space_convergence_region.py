'''
July 4 2025

If we can get the region of convergence for both cartesian and joint space, can we essentially get the closest vector to that space that successfully converges each time...?
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
from minimum_jacobian_convergence_regions_analytic import Trajectory, ConvergenceAlgorithm

def get_convergence_region(Trajectory: Trajectory):
    '''
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
            c=success_values,  # Color
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

    if Trajectory.planning_complete: #finished up our work here
        return 
    else:
        Trajectory.currMilestoneP = closest_success_point_to_starting_position
        return