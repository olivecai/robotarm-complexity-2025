'''
Aug 18 2025

Initialize a Jacobian of pure noise of values near 0, so that we can easily indicate which sign the slope should move towards.
Since the slope is near 0, the step we take must be extremely small.
'''

#initialize a matrix of random noise and broyden update it



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

print("ROBOT:\n",robot.J)

cam1 = dh.Camera(0,0,0,[0,0,2], 2,2, 0, 0) #looks down directly at the scene, basically replicates the scene
cam2 = dh.Camera(-sp.pi/2, 0, 0, [0,0,2], 2,2,0,0) #looks at scene from the y axis, world z is cam2 y, world x is cam2 x 
cameras=[cam1, cam2]

vs = dh.DenavitHartenberg_Cameras_Analytic(cameras, robot)
vs.dh_robot.alpha = 1.

kinova_angles = np.deg2rad(np.array([-0.1336059570312672, -28.57940673828129, -179.4915313720703, -147.7, 0.06742369383573531, -57.420898437500036, 89.88030242919922, 0.5]))
kinova_end = np.deg2rad(np.array([25.336059570312672, 50.57940673828129, -179.4915313720703, -90.7, 30.06742369383573531, -57.420898437500036, 30.88030242919922, 0.5]))

# initialize init Q and de
# initialize init Q and des P

initQ = [1.5] * robot.dof
desQ = [2.0] * robot.dof

desP =  (vs.dh_robot.fkin_eval(*desQ)).flatten().tolist()
print("desired world point:", desP)
print("initial world point:", vs.dh_robot.fkin_eval(*initQ).flatten().tolist())



def noisy(robot: dh.DenavitHartenberg_Cameras_Analytic, initQ, desP):
    '''
    Aug 18 2025

    Noisy jacobian refined by Broyden updates:

    Generate a Jacobian of noise.
    Then, at every step, perform Broyden's update to refine the Jacobian guess.

    How often should we update the Jacobian?
    How should we decide the step size?
    '''
    currQ = initQ
    tolerance = 1e-3
    maxiter=100

    traj = [currQ]
    errors =[]
    jac_update_iters =[]

    desPP = robot.projected_world_point(desP)
    print("LOG desPP", desPP)

    random_jac = 1
    if random_jac:
        p = 2*len(robot.cameras) # number of rows = 2*number of cameras, since each cam gives x and y 
        d = robot.dh_robot.dof #number of cols = robot joint dof
        low = -10
        high = 10

        J = np.random.uniform(low, high, size=(p, d))
    
        print("Randomized Jacobian:\n", J)
    else:
        J = vs.central_differences_pp(currQ, desPP)
        
    delQ = [0.]*robot.dh_robot.dof

    robot.calc_lipschitz(2.0) #calculate the lipschitz constant

    jac_count=1

    epsilon = 0.5 #the amount of error permitted from the amount of distance moved from where we did the jacobian initial estimation 


    jacobian_norm = np.linalg.norm(J)
    inv_jacobian_norm = np.linalg.norm(np.linalg.pinv(J))
    print("jac norm:", jacobian_norm)
    print("inv jac norm:", inv_jacobian_norm)
    condition_number = jacobian_norm * inv_jacobian_norm

    print("cond num:", condition_number)

    step = 1.0 #1/condition_number


    for i in range(maxiter):
        print("###################### i:", i)
       
        currError = robot.projected_errfn_eval(currQ, desPP) #desired-current
        prevError = currError.copy()

        if np.linalg.norm(currError) <= tolerance:
            break

        Jinv = np.linalg.pinv(J)

        corrQ = (Jinv @ currError).flatten()
        print(f"currQ: {currQ}\ncorrQ: {corrQ}\ncurrError: {currError}")
        
        delQ+=corrQ
        print("DELQ:", delQ)
        jacobian_approximated_error = (robot.lipschitz / 2) * (np.linalg.norm(delQ))**2
        
        #print("jac approximated err:", jacobian_approximated_error)

        jacobian_norm = np.linalg.norm(J)
        inv_jacobian_norm = np.linalg.norm(np.linalg.pinv(J))
        print("jac norm:", jacobian_norm)
        print("inv jac norm:", inv_jacobian_norm)
        condition_number = jacobian_norm * inv_jacobian_norm

        print("cond num:", condition_number)

        #step = 1/condition_number
        step=1.0
        
        prevQ = currQ
        currQ = currQ - step*corrQ
        traj.append(currQ)
        errors.append(np.linalg.norm(currError))

        currError = robot.projected_errfn_eval(currQ, desPP) #desired-current
        print("currError:", currError, "prevError:", prevError)
        print("currQ:", currQ, "prevQ:", prevQ, "corrQ:", corrQ)
        print(currError-prevError)

        #BROYDEN UPDATE:
        broyden=1
        dQ = np.subtract(currQ,prevQ)
        dError = np.subtract(currError,prevError)
        if broyden:
            broyden_numerator = np.multiply(np.reshape(np.subtract(dError, J@dQ),(-1,1)), dQ)
            broyden_denominator = np.array(dQ) @ np.array(dQ)
            jacobian_learning_rate=0.3
            J = J + jacobian_learning_rate * (broyden_numerator/broyden_denominator)
            print("Broyden update:\n", broyden_numerator/broyden_denominator)
            print("Updated Jacobian:\n", J)

    print("Total jacobians used:", jac_count)

    traj = np.array(traj)
    if 1: 
        robot.dh_robot.rtb_robot.plot(traj, block=False)
        robot.dh_robot.plot(currQ)

    errors=np.array(errors)
    fig, ax = plt.subplots()
    ax.plot(range(len(errors)), errors, '-o', markersize=2, label="Error")

    # Mark Jacobian update iterations with big red dots
    ax.scatter(jac_update_iters, errors[jac_update_iters], 
            s=50, c='blue', zorder=3, label="Jacobian Update")

    ax.set_xlabel("Iteration")
    ax.set_ylabel("Error Norm")
    ax.set_title("Error vs Iteration (Jacobian updates marked)")
    ax.legend()
    ax.grid(True)

    plt.show()

    return

kinova_angles = np.deg2rad(np.array([-0.1336059570312672, -28.57940673828129, -179.4915313720703, -147.7, 0.06742369383573531, -57.420898437500036, 89.88030242919922, 0.]))
# initQ = [0., 1.3]
#initQ = kinova_angles
# desP = [1.,1.,0]

robot.plot(initQ)

noisy(vs, initQ, desP)
