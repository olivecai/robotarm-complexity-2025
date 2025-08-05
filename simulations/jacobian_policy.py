'''
August 5 2025

This program uses tools like trust regions and Lipschitz constant
to make a policy on how big of a step we should take while ensuring our Jacobian is valid.

From the Numerical-Method-for-uncostrained-optimization-J.-E.-Dennis-Robert-B.-Schnabel (URL: Numerical-Method-for-uncostrained-optimization-J.-E.-Dennis-Robert-B.-Schnabel)
we have on page 74 of Chapter 4:

norm of ( F(x+p)-F(x)-J(x)p ) <= ( Lipschitz / 2 ) * norm(p)^2.

This inequality says: "The output change should correspond with the Jacobian*input change. This correspondence difference is bounded by the right hand term."

Ideally, the error in the output-input (LEFT HAND SIDE) should be decreasing with every iteration. 

Define an error policy that is evaluated at EVERY step. The policy will tell us:
- How big the step can be
- When the new Jacobian should be calculated
- If the trajectory will fail.

At the end of the trajectory we can count the number of Jacobians.
Perhaps different kinds of policies and constants can be run.

The number of jacobians will give us an idea for how nonlinear the function is.

It has been mentioned in literature that inverse kinematics algorithms are often robust to different camera positions, even if cameras are jostled after a specific camera position calibration.
If the function is very similar despite camera configurations, it could suggest that cameras do not greatly affect the underlying function.

We can look at the same task but with different (but equivalent) error constraints, and with different numbers of cameras and constraints, and see how the function improves.

This program does not make any convergence guarentees, and does not care whether or not we successfully converge. This is not a global convergence algorithm.
Instead, this program uses semilocal convergence as a measurement of nonlinearity.

Here, F is the error function, x is the current joint vector, p is the change in x AKA the correction step in the joint vector.

We want to see a decrease of error with every step, so at each update of x
'''

import denavit_hartenberg as dh
import sympy as sp
import numpy as np


P = dh.DHSympyParams() 
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
robot = dof2

cam1 = dh.Camera(0,0,0,[0,0,5], 5,5, 0, 0) #looks down directly at the scene, basically replicates the scene
cam2 = dh.Camera(-sp.pi/2, 0, 0, [0,0,5], 5,5,0,0) #looks at scene from the y axis, world z is cam2 y, world x is cam2 x 
cameras=[cam1, cam2]

vs = dh.DenavitHartenberg_Cameras_Analytic(cameras, robot)

def mod_newton_method1(robot: dh.DenavitHartenberg_Cameras_Analytic, initQ, desP):
    '''
    error_function has real world desired points and projects them, takes currQ
    '''
    currQ = initQ
    tolerance = 1e-3
    maxiter=50

    traj = [currQ]
    errors =[]

    desPP = robot.projected_world_point(desP)

    J = robot.central_differences_pp(currQ, desPP)  

    robot.calc_lipschitz() #calculate the lipschitz constant
    
    error_order_const = 0.8

    for i in range(maxiter):
        print("###################### i:", i)
        currError = robot.projected_errfn_eval(currQ, desPP) #desired-current

        if np.linalg.norm(currError) <= tolerance:
            break

        Jinv = np.linalg.pinv(J)

        corrQ = (Jinv @ currError).flatten()
        print(f"currQ: {currQ}\ncorrQ: {corrQ}\ncurrError: {currError}")

        numerator = 2 * error_order_const * np.linalg.norm(currError)
        denominator = robot.lipschitz * (np.linalg.norm(corrQ) ** 2)
        step = np.sqrt(numerator/denominator) #might have to bound this !!!

        print(f"numerator: {numerator}\ndenominator: {denominator}\nstep: {step}")

        print(f"currQ reg: {currQ-corrQ} VS currQ w step: {currQ-step*corrQ}")

        currQ = currQ - step * corrQ


        traj.append(currQ)
        errors.append(currError)


    traj = np.array(traj)
    if 1: 
        robot.dh_robot.rtb_robot.plot(traj, block=False)

    return



def mod_newton_method2(robot: dh.DenavitHartenberg_Cameras_Analytic, initQ, desP):
    '''
    TRUST REGIONS
    '''
    currQ = initQ
    tolerance = 1e-3
    maxiter=50

    traj = [currQ]
    errors =[]

    desPP = robot.projected_world_point(desP)

    J = robot.central_differences_pp(currQ, desPP)  

    robot.calc_lipschitz() #calculate the lipschitz constant
    
    step = 1

    dlower = 0.1
    dupper=0.7

    for i in range(maxiter):
        print("###################### i:", i)
        currError = robot.projected_errfn_eval(currQ, desPP) #desired-current

        if np.linalg.norm(currError) <= tolerance:
            break

        Jinv = np.linalg.pinv(J)

        corrQ = (Jinv @ currError).flatten()
        print(f"currQ: {currQ}\ncorrQ: {corrQ}\ncurrError: {currError}")

        d = np.linalg.norm(currError) / np.linalg.norm(corrQ)        

        print(f"d: {d}")

        if d <= dlower:
            step = step/2
        elif d > dupper:
            step = max(2*np.linalg.norm(corrQ), step)
        #else, step remains the same as the previous iteration.

        print(f"step: {step}")

        print(f"currQ reg: {currQ-corrQ} VS currQ w step: {currQ-step*corrQ}")

        currQ = currQ - step * corrQ


        traj.append(currQ)
        errors.append(currError)


    traj = np.array(traj)
    if 1: 
        robot.dh_robot.rtb_robot.plot(traj, block=False)

    return

 

def mod_newton_method3(robot: dh.DenavitHartenberg_Cameras_Analytic, initQ, desP):
    '''
    error_function has real world desired points and projects them, takes currQ
    '''
    currQ = initQ
    tolerance = 1e-3
    maxiter=50

    traj = [currQ]
    errors =[]

    desPP = robot.projected_world_point(desP)

    J = robot.central_differences_pp(currQ, desPP)  

    robot.calc_lipschitz(3.97) #calculate the lipschitz constant
    
    error_order_const = 0.8

    jac_count = 0

    for i in range(maxiter):
        print("###################### i:", i)
        currError = robot.projected_errfn_eval(currQ, desPP) #desired-current

        if np.linalg.norm(currError) <= tolerance:
            break

        Jinv = np.linalg.pinv(J)

        corrQ = (Jinv @ currError).flatten()
        print(f"currQ: {currQ}\ncorrQ: {corrQ}\ncurrError: {currError}")
        
        numerator = 2 * error_order_const * np.linalg.norm(currError)
        denominator = robot.lipschitz * (np.linalg.norm(corrQ) ** 2)
        step = np.sqrt(numerator/denominator) #might have to bound this !!!

        print(f"numerator: {numerator}\ndenominator: {denominator}\nstep: {step}")

        leftside = (robot.lipschitz / 2) * (np.linalg.norm(step*corrQ))**2
        rightside = error_order_const * np.linalg.norm(currError)

        jac_good = leftside <= rightside
        print(f"inequality: {leftside} <= {rightside}. BOOL=={jac_good}")

        if step<1:
            J = robot.central_differences_pp(currQ, desPP)
            jac_count+=1


        currQ = currQ - step* corrQ


        traj.append(currQ)
        errors.append(currError)

    
    print(f"jacobian count: {jac_count}")


    traj = np.array(traj)
    if 1: 
        robot.dh_robot.rtb_robot.plot(traj, block=False)
        robot.dh_robot.plot(currQ)
    return

kinova_angles = np.deg2rad(np.array([-0.1336059570312672, -28.57940673828129, -179.4915313720703, -147.7, 0.06742369383573531, -57.420898437500036, 89.88030242919922, 0.]))
initQ = [-3.14, 1.5]
desP = [0.,0.3,0.]

robot.plot(initQ)

mod_newton_method3(vs, initQ, desP)

'''
Notes:

Reducing the step size can only go so far. We have to know when to reduce step size and when to update the Jacobian.

Lipschitz Constants:
- kinova and 2 cameras: 3.97
'''