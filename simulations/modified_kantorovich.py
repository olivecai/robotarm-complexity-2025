'''
July 23 2025

This program uses Kantorovich principles in an algorithm designed to:
- use as few jacobians as possible to complete the inverse kinematics
- move out of singular positions

TODO: need to implement constrained nonlinear system as unconstrained with penalty.

In contrast to the heavy theory present in Kantorovich's theorem, we will be using very empirical 'quick fixes' in this program, 
just to visualize what might work or not.

So, what are 'quick fixes' that we think could work based on our 2,3, kinova-dof?

- h must be less than 4 or so, maybe 5... (according to kantorovich, it should be less than 1/2, but that is too low)
- closer we are == dampening closer to 1 (farther, dampening closer to 0)
- instead of using the global lipschitz constant, use the spectral radius at that points (possible multiply by a factor a little greater than 1)
- b must be less than 1 or 1.5 ish --> cannot be too far.

Are there cases when b is HIGHER (significantly higher) and we can still converge?

And for all of the above, let them be guidance, or suggestions, but provide proper fallback if the method begins to fail.]
This way we don't need to be as worried about failure.

if B is very large, then we are at a singularity and should move out of that configuration.


This program partially exists as a last ditch effort to not give up on the Kantorovich semilocal convergence...

'''

import denavit_hartenberg as dh
import sympy as sp
import numpy as np
import matplotlib.pyplot as plt

import denavit_hartenberg as dh

####################################
##### SET UP PARAMS AND ROBOT ######
#######################################

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
robot = dof3

#############################################333

def convergence_conditions(h, b, B):
    #Assume that we have bounds for h, b, B. This step feels silly right now, but if we can figure out equations for these bounds, it's not so bad.
    h_upper_kantorovich_bound = 0.5
    h_upper_bound = 5.
    b_upper_bound = 2.
    B_lower_bound = 200.

    estimated_jacobians = 0

    if h < h_upper_kantorovich_bound: # by kantorovich
        estimated_jacobians = 1
    elif h < h_upper_bound and b < b_upper_bound: # by empirical/guessing
        estimated_jacobians = 1 
    else:
        estimated_jacobians = "Some More"
    
    if B > B_lower_bound: #singular
        estimated_jacobians = "SINGULAR"
        print("SINGULAR POSITION")

    return estimated_jacobians

def modified_kantorovich_invkin(robot: dh.DenavitHartenbergAnalytic, initQ, desP, alpha, lipschitz=None):
    alpha=1.0
    h, p, b, B = robot.ret_kantovorich(initQ=initQ, desP=desP, alpha=alpha, lipschitz=None, )

    estimated_jacobians = convergence_conditions(h, b, B)
    result_jacobian_count= 1

    # compute the inverse kinematics and return the number of jacobians so we can compare after
    currQ = initQ
    tolerance = 1e-3
    maxiter= 200

    traj = [currQ]
    
    J = robot.central_differences(currQ, desP)

    ret= -1
    F = robot.F
    reps_des = []
    for i in range(len(desP)):
        reps_des.append((robot.cartvars[i], desP[i]))
    F = F.subs(reps_des)
    
    prevError = np.inf
    consecutive_error_increases = 0

    print("### CALIBRATED JACOBIAN ###\nCurrently on jacobian", result_jacobian_count)

    for i in range(maxiter):

        reps_dof = []
        
        for j in range(robot.dof):
            reps_dof.append((robot.jntvars[j], currQ[j]))

        currError = np.array(F.subs(reps_dof).evalf()).astype(np.float64)
        #print("currError:", currError.flatten())

        if np.linalg.norm(currError) <= tolerance:
            ret = i
            break
        
        newtonStep = (np.linalg.pinv(J) @ currError).flatten()
        print("i", i,"\ncurrQ", currQ)
        currQ = currQ - robot.alpha * newtonStep
        #print("currQ:", currQ)
        traj.append(currQ)

        if np.linalg.norm(currError) > np.linalg.norm(prevError):
            consecutive_error_increases+=1

        if consecutive_error_increases > 3:
            J = robot.central_differences(currQ, desP)
            result_jacobian_count+=1
            print("### RECALIBRATED JACOBIAN ###\nCurrently on jacobian", result_jacobian_count)
            consecutive_error_increases = 0

        prevError= currError

    print("Finished. i:", ret, "error:", currError)
    print("Estimated jacobians:", estimated_jacobians, "Actual jacobians:", result_jacobian_count)

    traj=np.array(traj)
    if 1:
        robot.rtb_robot.plot(traj, block=False)

################################################

'''
complete inverse kinematics:
'''
desQ = [np.pi/4] * robot.dof
desP = robot.fkin_eval(*desQ).flatten().tolist()
print("desQ:", desQ, "\ndesP:\n", desP)
initQ = [0.4] * robot.dof
    
# Choose a starting position and the error function to minimize...
robot.plot(desQ)
robot.plot(initQ)

alpha=0.01
h, p, b, B = robot.ret_kantovorich(initQ=initQ, desP=desP, alpha=alpha, lipschitz=None, )

modified_kantorovich_invkin(robot, initQ, desP, 1)

