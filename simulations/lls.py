'''
August 24 2025

Implementation of LLS method of "Global Visual-Motor Estimation for Uncalibrated Visual Servoing"
https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=4399506

Question: how many samples of Jacobians are needed to reduce the Jacobian estimation error between different dofs, 
and what does this say about the complexity?

Choose k to be constant in trial 1.

Then choose an adaptive k so that if the neighbouring jacobians are very different (indicating nonlinearity), we increase OR decrease k (we can do either)
(maybe see if the 5 closest points live on the same hyperplane and if NOT then we are likely in quite a nonlinear region)

Choose a dampening step that is low, maybe 0.2, to replicate the way that the real control feedback loop would work. 

1. Generate N random datapoints of joint configurations. This is our offline acquisition of datapoints. Start at 100 datapoints, increase by 100 and repeat the below:
2. Run 10 different, random, smooth joint configurations, and output the error between the true vs the estimated Jacobian estimation for that point.

Hypothesis:
- 2 dof jacobian estimation error will decrease very fast the more jacobians we sample beforehand.
- 3 dof it will decrease slower but still very fast.
- 7 dof it will decrease slowest.

This is very straightforward of course, but perhaps there exists some relationship between how many jacobians are needed for the comparison.

Plotting this may be a challenge so make sure we print out the datapoints so we can cache them and plot it later.

TODO as a next step:
- Compare k constant vs k adaptive for the nearest neighbours to use in the LLS.

TODO AFTER
Then, now that we have this offline joint config map, 
can we employ some strategy to pick and choose the 'best jacobians' as we travel? We know tha

'''

import numpy as np
import sympy as sp
import denavit_hartenberg as dh
import matplotlib.pyplot as plt

P = dh.DHSympyParams() #this lowkey has to be global, sorry
jntspace, cartspace, taskspace = P.get_params()
t0, t1, t2, t3, t4, t5, t6, t7, t8, t9 = jntspace
x, y, z = cartspace
u, v = taskspace

dof2_params = [
                [t0, 0, 1, 0], 
                [t1, 0, 1, 0]
                ]

dylan_dof3_params=[
                [ t0, sp.pi/2, 0 , 0 ],
                [ t1,  0  ,  0.55, 0 ],
                [ t2,  0  ,  0.3, 0 ]
                ]


kinova_dof7_params = [
    [t0,      sp.pi,   0.0,   0.0],
    [t1,      sp.pi/2, 0.0, -(0.1564 + 0.1284)],
    [t2 +sp.pi, sp.pi/2, 0.0, -(0.0054 + 0.0064)],
    [t3 +sp.pi, sp.pi/2, 0.0, -(0.2104 + 0.2104)],
    [t4 +sp.pi, sp.pi/2, 0.0, -(0.0064 + 0.0064)],
    [t5 +sp.pi, sp.pi/2, 0.0, -(0.2084 + 0.1059)],
    [t6 +sp.pi, sp.pi/2, 0.0, 0.0],
    [t7 +sp.pi,    sp.pi, 0.0, -(0.1059 + 0.0615)],
]

dof2 = dh.DenavitHartenbergAnalytic(dof2_params, P)
dof3 = dh.DenavitHartenbergAnalytic(dylan_dof3_params, P)
kinova = dh.DenavitHartenbergAnalytic(kinova_dof7_params, P)
robot = dof3

# print("ROBOT:\n",robot.J_analytic)

cam1 = dh.Camera(0,0,0,[0,0,2], 2,2, 0, 0) #looks down directly at the scene, basically replicates the scene
cam2 = dh.Camera(-sp.pi/2, 0, 0, [0,0,2], 2,2,0,0) #looks at scene from the y axis, world z is cam2 y, world x is cam2 x 
cameras=[cam1, cam2]

vs = dh.DenavitHartenberg_Cameras_Analytic(cameras, robot)
vs.dh_robot.alpha = 1.

kinova_angles = np.deg2rad(np.array([-0.1336059570312672, -28.57940673828129, -179.4915313720703, -147.7, 0.06742369383573531, -57.420898437500036, 89.88030242919922, 0.5]))
kinova_end = np.deg2rad(np.array([25.336059570312672, 50.57940673828129, -179.4915313720703, -90.7, 30.06742369383573531, -57.420898437500036, 30.88030242919922, 0.5]))

# initialize init Q and de
# initialize init Q and des P


jointranges = [(-np.pi/2, np.pi/2)]*robot.dof
initQ = [0.5] * robot.dof
desQ = [0.] * robot.dof

desP =  (vs.dh_robot.fkin_eval(*desQ)).flatten().tolist()
desPP = vs.projected_world_point(desP)
# print("desired world point:", desP)
# print("initial world point:", vs.dh_robot.fkin_eval(*initQ).flatten().tolist())


def generate_random_trajectory(vs: dh.DenavitHartenberg_Cameras_Analytic, jointranges: list, number_of_milestone_points=2, number_of_transition_points=2):
    '''
    Choose a random trajectory
    '''

    joint_configs = [
            np.array([np.random.uniform(low, high) for (low, high) in jointranges])
            for _ in range(number_of_milestone_points)
        ]
    
    # print("randomly generated joint configs:", joint_configs)
    
    trajectory = []
    for i in range(len(joint_configs)-1):
        a = joint_configs[i]
        b= joint_configs[i+1]

        transition = np.linspace(a,b,number_of_transition_points)

        trajectory.append(a.tolist())
        for p in transition.tolist():
            trajectory.append(p)

        trajectory.append(b.tolist())

    # vs.dh_robot.plot(np.array(trajectory))
    return trajectory

def n_samples(vs: dh.DenavitHartenberg_Cameras_Analytic, jointranges: list, n: int):
    # joints generated, points generated
    points_generated = []
    joints_generated = [
            np.array([np.random.uniform(low, high) for (low, high) in jointranges])
            for _ in range(n)
        ]
    for Q in joints_generated:
        real_point = vs.dh_robot.fkin_eval(*Q)
        points_generated.append(vs.projected_world_point(real_point))
    return joints_generated, points_generated


class LLS_VS:
    def __init__(self, vs: dh.DenavitHartenberg_Cameras_Analytic, jointranges: list, n: int, k: int = None):
        self.n = n
        self.k = k
        # self.trajectory = generate_random_trajectory(vs, jointranges)
        
        self.vs = vs
        self.jointranges=jointranges

    def get_approx_jac(self, Q):
            Y = self.vs.projected_world_point(self.vs.dh_robot.fkin_eval(*Q))
        
            distances = [np.linalg.norm(np.subtract(Q, storedQ)) for storedQ in self.joints_generated]
            # Get indices of the k smallest distances
            nearest_indices = np.argsort(distances)[:k]
            nearest_Q_distances = np.array(distances)[nearest_indices]
            nearest_points = np.array([self.points_generated[i] for i in nearest_indices])
            nearest_Q = np.array([self.joints_generated[i] for i in nearest_indices])

            weights = [i**-1 for i in nearest_Q_distances]
            sum_distances = np.sum(weights)
            weights = weights/sum_distances

            # dQ = np.zeros((k * k, self.vs.dh_robot.dof))
            # dY = np.zeros((k * k, Y.shape[0]))
            # for i in range(k):
            #     for j in range(k):
            
            #         dQ[i * k + j, :] = nearest_Q[i, :] - nearest_Q[j, :]
            #         dY[i * k + j, :] = nearest_points[i, :] - nearest_points[j, :]

     
            dQ = np.vstack([np.subtract(nearby_Q, Q) for nearby_Q in nearest_Q])
            dQ = np.vstack([w * dq for w, dq in zip(weights, dQ)])
            
            dY = np.vstack([np.subtract(nearby_pnt, Y) for nearby_pnt in nearest_points])
            dY = np.vstack([w * dy for w, dy in zip(weights, dY)])

            W = np.diag(weights)

            # print(f"Q: {Q}")
            # print(f"nearestQ: {nearest_Q}")

            # print(f"Y: {Y}")
            # print(f"nearest Y: {nearest_points}")
            # print(f"dQ: {dQ}")
            # print(f"dY: {dY}")

            XtX = dQ.T @ W @ dQ

            # print("XtX", XtX)
            
            XtY = dQ.T @ W @ dY

            # print("XtY", XtY)

            approxJ = (np.linalg.pinv(XtX) @ XtY).T  # shape: (error_dim, dof)

            # #2) robust solve via lstsq (J^T is solution)
            # Jt, *_ = np.linalg.lstsq(dQ, dY, rcond=None)
            # approxJ = Jt.T  # (m, dof)

            return approxJ


    def monitor_true_vs_lls(self, random_trajectory: list, k=None):
        '''
        as we move through one randomly generated trajectory, we can sum the error from the true vs approximated jacobian 
        and return the mean amount of jacobian error squared

        mean of error norm 
        '''
        print("n, k:", self.n, self.k)
        total_norm_error = 0
        number_of_jacobians_compared = 0
        mean_norm_error = 0 # error squared / number of jac TRUE vs APPROX 

        if k==None:
            k = self.k

        # for each Q in the joint trajectory... compare its true Jacobian VS the LLS:
        for Q in random_trajectory:
                
            number_of_jacobians_compared += 1
            #print("########## ITERATION", number_of_jacobians_compared, "##################")

            # print(desPP)
            # print(Q)
            true_cdJ  = -self.vs.central_differences_pp(Q, desPP)

            approxJ = self.get_approx_jac(Q)

            #print(f"Jacobian Comparison:\nTrue Jacobian:\n{true_cdJ}\nApprox Jacobian:\n{approxJ}")

            Jerrornorm = np.linalg.norm((np.subtract(approxJ,true_cdJ)))
            #print(f"norm error between true and approx J: {Jerrornorm}")

            
            total_norm_error += Jerrornorm
            
        mean_norm_error = total_norm_error / number_of_jacobians_compared

        print("mean norm error:", mean_norm_error)

        return mean_norm_error


    def jacobian_estimation_error_as_N_grows(self, N):
        
        error_at_N = []
        for n in N:
            mean_over_means = 0
            mean_over_means_count = 10
            num_checkpnts = 25
            num_transitions = 1
            for _ in range(mean_over_means_count):
                self.joints_generated, self.points_generated = n_samples(self.vs, self.jointranges, n)
                self.n=n
                traj = generate_random_trajectory(self.vs, self.jointranges, num_checkpnts, num_transitions)
                mean_norm_error = self.monitor_true_vs_lls(traj)
                mean_over_means+=mean_norm_error
            error_at_N.append(mean_over_means/mean_over_means_count)
        
        print("N:", N)
        print("error at N:", error_at_N)
        return(error_at_N)
        # plt.plot(N, error_at_N)
        # plt.xlabel("N")
        # plt.ylabel("Jacobian Estimation Mean Norm Error")
        # plt.title(f"{self.vs.dh_robot.dof} DOF Robot")
        # plt.show()

def different_dofs_vs_N():
    testsystems = dof2, dof3, kinova

    cam1 = dh.Camera(0,0,0,[0,0,2], 2,2, 0, 0) #looks down directly at the scene, basically replicates the scene
    cam2 = dh.Camera(-sp.pi/2, 0, 0, [0,0,2], 2,2,0,0) #looks at scene from the y axis, world z is cam2 y, world x is cam2 x 
    cameras=[cam1, cam2]

    testsystems_N_error_values = []

    N = [i for i in range(200,5100,200)]
    
    for i in range(len(testsystems)):
        robot= testsystems[i]

        print(f"********** ROBOT TEST SYSTEM {i}")
        vs = dh.DenavitHartenberg_Cameras_Analytic(cameras, robot)
        vs.dh_robot.alpha = 1.

        n=None
        k=50

        jointranges = [(-np.pi/2, np.pi/2)]*robot.dof
        desQ = [0.] * robot.dof

        jointranges = [(-np.pi/2, np.pi/2)]*robot.dof
        desP =  (vs.dh_robot.fkin_eval(*desQ)).flatten().tolist()
        desPP = vs.projected_world_point(desP)

        lls_vs = LLS_VS(vs, jointranges, n, k)
        error_at_N = lls_vs.jacobian_estimation_error_as_N_grows(N)
        testsystems_N_error_values.append(error_at_N)
    
    plt.plot(N, testsystems_N_error_values[0], label='dof1', color='blue')
    plt.plot(N, testsystems_N_error_values[1], label='dof2', color='red')
    plt.plot(N, testsystems_N_error_values[2], label='kinova', color='green')
    plt.xlabel("N")
    plt.ylabel("Jacobian Estimation Mean Norm Error")
    plt.title(f"Measure of Jacobian Estimation Error for Robot System VS N")
    plt.legend()
    plt.show()


# trajectory = generate_random_trajectory(vs, jointranges)

n = 1000
k = 10

lls_vs = LLS_VS(vs, jointranges, n, k)
# random_traj = generate_random_trajectory(vs, jointranges)
# lls_vs.monitor_true_vs_lls(random_traj)
# lls_vs.jacobian_estimation_error_as_N_grows()
different_dofs_vs_N()