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
robot = kinova

# print("ROBOT:\n",robot.J_analytic)

cam1 = dh.Camera(0.1,0.05,0,[0,0,4], 4,4, 0, 0) 
cam2 = dh.Camera(-sp.pi/2, 0, 0.5, [0,0,4], 4,4,0,0) #looks at scene from the y axis, world z is cam2 y, world x is cam2 x 
cameras=[cam1, cam2]

vs = dh.DenavitHartenberg_Cameras_Analytic(cameras, robot)
vs.dh_robot.alpha = 1.


kinova_angles = np.deg2rad(np.array([-0.1336059570312672, -28.57940673828129, -179.4915313720703, -147.7, 0.06742369383573531, -57.420898437500036, 89.88030242919922, 0.5]))



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
    # print("joints gen", joints_generated,"\n joint gen fin")
    for Q in joints_generated:
        real_point = vs.dh_robot.fkin_eval(*Q)
        points_generated.append(vs.projected_world_point(real_point))
    return joints_generated, points_generated


class LLS_VS:
    def __init__(self, vs: dh.DenavitHartenberg_Cameras_Analytic, jointranges: list, n: int = None, k: int = None):
        self.n = n
        self.k = k
        # self.trajectory = generate_random_trajectory(vs, jointranges)
        self.Lref = None
        self.vs = vs
        self.joints_generated, self.points_generated = None, None
        self.jointranges=jointranges

    def get_approx_jac(self, Q):
            Y = self.vs.projected_world_point(self.vs.dh_robot.fkin_eval(*Q))
        
            distances = [np.linalg.norm(np.subtract(Q, storedQ)) for storedQ in self.joints_generated]
            # Get indices of the k smallest distances
            print("this is k:", self.k)
            nearest_indices = np.argsort(distances)[:self.k]
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
           
            dY = np.vstack([np.subtract(nearby_pnt, Y) for nearby_pnt in nearest_points])
           
            # W = np.diag(weights)

            # print(f"Q: {Q}")
            # print(f"nearestQ: {nearest_Q}")

            # print(f"Y: {Y}")
            # print(f"nearest Y: {nearest_points}")
            # print(f"dQ: {dQ}")
            # print(f"dY: {dY}")

            XtX = dQ.T @ dQ

            # print("XtX", XtX)
            
            XtY = dQ.T @ dY

            # print("XtY", XtY)

            approxJ = ((np.linalg.pinv(XtX) @ XtY).T) # shape: (error_dim, dof)
         
            # #2) robust solve via lstsq (J^T is solution)
            # Jt, *_ = np.linalg.lstsq(dQ, dY, rcond=None)
            # approxJ = Jt.T  # (m, dof)

            return -approxJ #something somewhere got messed up and i think my world coords r inverted, so this needs neg


    def monitor_true_vs_lls(self, random_trajectory: list, k=None):
        desPP = np.zeros(2*len(self.vs.cameras))
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
            
            if self.Lref:
                true_cdJ  = self.vs.central_differences_pp(Q, desPP)/self.Lref
                approxJ = self.get_approx_jac(Q)/self.Lref
            else:
                true_cdJ  = self.vs.central_differences_pp(Q, desPP)
                approxJ = self.get_approx_jac(Q)


            print(f"Jacobian Comparison:\nTrue Jacobian:\n{true_cdJ}\nApprox Jacobian:\n{approxJ}")

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
            num_checkpnts = 10
            num_transitions = 3
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

# Q = [0.,1.5,1.5]
# jointranges = [(-0.1,0.1),(1.4,1.6),(1.4,1.6)]
# lls_vs = LLS_VS(vs, jointranges, None, 100)
# lls_vs.joints_generated, lls_vs.points_generated = n_samples(vs, jointranges, 100)
# apprx = lls_vs.get_approx_jac(Q)
# true =-vs.central_differences_pp(Q, vs.projected_world_point(vs.dh_robot.fkin_eval(*Q)))
# print(apprx,"\n", true)
# print(np.subtract(apprx, true))
# print(np.linalg.norm(np.subtract(true, apprx)))

# if 0:
#     exit(0)


def visual_servo_kinova(lls_vs : LLS_VS, n, k, initQ, desP, mode):
    '''
    mode:
    0 == update the jacobian at every step (control)
    1 == just one jacobian (???)
    2 == different values of n, hold k constant
    3 == strategic choose k, choose a suitable constant n
    '''
    robot = lls_vs.vs
    if mode:
        lls_vs.n = n
        lls_vs.k = k
        lls_vs.joints_generated, lls_vs.points_generated = n_samples(lls_vs.vs, lls_vs.jointranges, n)

    currQ = initQ
    tolerance = 1e-3
    maxiter=50

    traj = [currQ]
    errors =[]

    desPP = robot.projected_world_point(desP)

    if not mode:
        J = robot.central_differences_pp(currQ, desPP)  
    else:
        J = lls_vs.get_approx_jac(currQ)

    alpha = 0.1

    for i in range(maxiter):
        print("###################### i:", i)
        currError = robot.projected_errfn_eval(currQ, desPP) #desired-current

        if np.linalg.norm(currError) <= tolerance:
            break

        Jinv = np.linalg.pinv(J)

        corrQ = (Jinv @ currError).flatten()
        print(f"currQ: {currQ}\ncorrQ: {corrQ}\ncurrError: {currError}")


        currQ = currQ - alpha * corrQ

        if not mode:
            J = robot.central_differences_pp(currQ, desPP)  
        elif mode >= 2: #skip jac update for 1
            J = lls_vs.get_approx_jac(currQ)
            print("approx:",J)
            print("true:", robot.central_differences_pp(currQ, desPP)  )

        traj.append(currQ)
        errors.append(np.linalg.norm(currError))

    traj = np.array(traj)
    if 1: 
        robot.dh_robot.rtb_robot.plot(traj, block=True)

    print(f"n:{n}, k:{k}, initQ: {initQ}, desP: {desP}, mode: {mode}\nerror from goal during visual servoing:\n{errors}")
    print(f"real world final position (just curious) {(robot.dh_robot.fkin_eval(*currQ))}")

    return errors


def kinova_vs_N():
    robot = kinova

    cam1 = dh.Camera(0,0,0,[0,0,2], 2,2, 0, 0) #looks down directly at the scene, basically replicates the scene
    cam2 = dh.Camera(-sp.pi/2, 0, 0, [0,0,2], 2,2,0,0) #looks at scene from the y axis, world z is cam2 y, world x is cam2 x 
    cameras=[cam1, cam2]

    N = [i for i in range(200, 5100, 200)]
    
    print(f"********** DOF3")

    vs = dh.DenavitHartenberg_Cameras_Analytic(cameras, robot)
    vs.dh_robot.alpha = 1.

    n=None
    k=40

    desQ = [0.] * robot.dof

    jointranges = [(-np.pi/2, np.pi/2)]*robot.dof
    desP =  (vs.dh_robot.fkin_eval(*desQ)).flatten().tolist()
    desPP = vs.projected_world_point(desP)

    lls_vs = LLS_VS(vs, jointranges, n, k)
    errors_at_N = lls_vs.jacobian_estimation_error_as_N_grows(N)
    print("errors at N:", errors_at_N)

    plt.plot(N, errors_at_N, label='kinova', color='blue')
    plt.xlabel('N')
    plt.ylabel('Jacobian Estimation Mean Norm Error')
    plt.show()
    


def different_dofs_vs_N():
    testsystems = dof2, dof3, kinova

    cam1 = dh.Camera(0,0,0,[0,0,2], 2,2, 0, 0) #looks down directly at the scene, basically replicates the scene
    cam2 = dh.Camera(-sp.pi/2, 0, 0, [0,0,2], 2,2,0,0) #looks at scene from the y axis, world z is cam2 y, world x is cam2 x 
    cameras=[cam1, cam2]

    testsystems_N_error_values = []

    N = [i for i in range(200,1300, 200)]
    
    for i in range(len(testsystems)):


        robot= testsystems[i]

        L_ref = sum(abs(link.a) + abs(link.d) for link in robot.rtb_model().links)


        print(f"********** ROBOT TEST SYSTEM {i}")
        print("approximate reach:", L_ref)

        vs = dh.DenavitHartenberg_Cameras_Analytic(cameras, robot)
        vs.dh_robot.alpha = 1.

        n=None
        k=40

        jointranges = [(-np.pi/2, np.pi/2)]*robot.dof
        desQ = [0.] * robot.dof

        jointranges = [(-np.pi/2, np.pi/2)]*robot.dof
        desP =  (vs.dh_robot.fkin_eval(*desQ)).flatten().tolist()
        desPP = vs.projected_world_point(desP)

        lls_vs = LLS_VS(vs, jointranges, n, k)
        lls_vs.Lref = L_ref
        error_at_N = lls_vs.jacobian_estimation_error_as_N_grows(N)
        testsystems_N_error_values.append(error_at_N)


    
    
    plt.plot(N, testsystems_N_error_values[0], label='dof1', color='blue')
    plt.plot(N, testsystems_N_error_values[1], label='dof2', color='red')
    plt.plot(N, testsystems_N_error_values[2], label='kinova', color='green')
    plt.xlabel("Iteration")
    plt.ylabel("Jacobian Estimation Mean Norm Error")
    plt.title("Jacobian Estimation Error vs Iteration Count for Robot System")
    plt.legend()
    plt.show()

def plot_simple():
    N = [200, 400, 600, 800, 1000, 1200, 1400, 1600, 1800, 2000, 2200, 2400, 2600, 2800, 3000, 3200, 3400, 3600, 3800, 4000, 4200, 4400, 4600, 4800, 5000]
    errorvals= [0.9155693903296006, 0.7902768927199364, 0.7541904060943748, 0.6849275989709345, 0.7260755849381872, 0.7117420644108934, 0.6868875815966596, 0.6452917772377391, 0.6666451367906865, 0.6487818144914833, 0.6324925152648528, 0.6288904785357227, 0.5981387528273808, 0.6391226044697526, 0.6083390282531584, 0.5913815172534631, 0.5984999910935305, 0.568823224002408, 0.6097930612106808, 0.5694784492173494, 0.56376947210882, 0.5751507704892037, 0.5778280407614825, 0.5741164277025658, 0.5741701981284801]
    plt.plot(N, errorvals, label='kinova', color='green')
    plt.xlabel('N')
    plt.ylabel('Jacobian Estimation Mean Norm Error')
    plt.show()

jointranges = [(-np.pi, np.pi)]*robot.dof

# trajectory = generate_random_trajectory(vs, jointranges)

n = None
k = None


if robot == kinova:
    pi=np.pi
    jointranges = [(-pi,pi)]*robot.dof

lls_vs = LLS_VS(vs, jointranges, n, k)
# random_traj = generate_random_trajectory(vs, jointranges)
# lls_vs.monitor_true_vs_lls(random_traj)
# lls_vs.jacobian_estimation_error_as_N_grows()
# different_dofs_vs_N()

# kinova_vs_N()
# plot_simple()

N = [n for n in range(100,5100,500)]
mode = 2 #0 or 2
errors=[]


# desQ = [0.4] * robot.dof

# desP =  (vs.dh_robot.fkin_eval(*desQ)).flatten().tolist()
# desPP = vs.projected_world_point(desP)
# print("desired world point:", desP)
# print("initial world point:", vs.dh_robot.fkin_eval(*initQ).flatten().tolist())

initQ = np.deg2rad(np.array([-0.1336059570312672, -28.57940673828129, -179.4915313720703, -147.7, 0.06742369383573531, -57.420898437500036, 89.88030242919922, 0.5]))
desP = []
desQ = np.deg2rad(np.array([-0.1336059570312672, -10.57940673828129, -180.4915313720703, -20.7, 50.06742369383573531, -10.420898437500036, 89.88030242919922, 0.5]))

print("WORLD INIT", (robot.fkin_eval(*initQ)))
print("PROJECTED INIT", vs.projected_world_point(robot.fkin_eval(*initQ)))
robot.plot(initQ)
# print("WORLD DES", (robot.fkin_eval(*desQ)))
# print("PROJECTED DES", vs.projected_world_point(robot.fkin_eval(*desQ)))
robot.plot(desQ)

if not mode:
    error = visual_servo_kinova(lls_vs, None, None, initQ, desP, mode)
    errors.append(error)
# mode+=1
if mode:
    for n in N:
        error = visual_servo_kinova(lls_vs, n, 20, initQ, desP, mode)
        errors.append(error)

maxiterscount=0
for error in errors:
    maxiterscount=max(maxiterscount, (len(error)))

print("errors", errors)
print("#### \nfor error in errors print error")
for error in errors:
    print(error)

plt.close('all')

# plt.plot(range(maxiterscount), errors[0], label='kinova', color='red')
# plt.xlabel("N")
# plt.ylabel("Jacobian Estimation Mean Norm Error")
# plt.title("Measure of Jacobian Estimation Error for Robot System VS N")

# plt.show()


for i in range(len(errors)):
    error=errors[i]
    
    plt.close('all')
    plt.plot(range(len(error)), error, label='kinova', color='red')
    plt.xlabel("N")
    plt.ylabel("Jacobian Estimation Mean Norm Error")
    plt.title("Measure of Jacobian Estimation Error for Robot System VS N")

    plt.show()









'''
Questions:
- why doesnt the plot work as well for the 2dof arm?
for adaptive K:
- check if the 
- 
'''