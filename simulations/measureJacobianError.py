'''
August 11 2025

OBJECTIVE:
Measure how much error our current central differences jacobian will tolerate.

The truncation error in the Taylor series for central differences jacobian is O(h^2).
So, if we have h<1, it's likely that the error will be VERY SMALL.
Specifically the truncation error of the Jacobian is (f(3)(x) * h^2 / 6) + O(h^4), where f(3) is the third derivative of the function.

We dont know the third derivative of the function, but we can assume that it is bounded by some constant M.
How to get M? TODO figure this out



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
                [t0, 0, 100, 0], 
                [t1, 0, 10, 0]
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

initQ = [1.0] * robot.dof
desQ = [1.5] * robot.dof

desP =  (vs.dh_robot.fkin_eval(*desQ)).flatten().tolist()
print("desired world point:", desP)
print("initial world point:", vs.dh_robot.fkin_eval(*initQ).flatten().tolist())


def plot_projected_scene(cameras, eeP, desP):
    '''
    For every camera, generate a window to plot the end effector (blue) and the desired point (red)
    Label each camera plot with the camera index.
    '''
    num_cams = len(cameras)
    cols = 2
    rows = (num_cams + 1) // cols

    fig, axs = plt.subplots(rows, cols, figsize=(8, 4 * rows))
    axs = axs.flatten()

    for i in range(num_cams):
        cam_i_desP = cameras[i].projectpoint(desP)    
        cam_i_eeP = cameras[i].projectpoint(vs.dh_robot.fkin_eval(*initQ))

        ax = axs[i]
        ax.scatter(*cam_i_desP[:2], color='green', label='Desired Point')
        ax.scatter(*cam_i_eeP[:2], color='red', label='EE Position')
        ax.set_title(f'Camera {i}')
        ax.set_xlim(-3, 3)
        ax.set_ylim(-3, 3)
        ax.set_aspect('equal')
        ax.grid(True)
        ax.legend()

    # Hide any unused subplots
    for j in range(i+1, len(axs)):
        fig.delaxes(axs[j])

    plt.tight_layout()
    plt.show()


# evaluate Kantorovich conditions
def eval_kantorovich_conditions(F, J, beta_preset: float=0.00, L=None):
    '''
    evaluate based on Kantorovich conditions whether convergence is possible...

    beta is traditionally = norm of the inverse jacobian as the measure of how singular the fucntion is.
    if beta > 0 is given as a float, use a CONSTANT beta
    if beta == 0 , calculate the beta each time.
    if beta < 0, log10 transform

    L is the Lipschitz constant. It is difficult to calculate globally.
    if L == None, approximate the glboal lipschitz constant as the local spectral norm of J
    if L is given, use L. (maybe global Lipschitz)

    Different papers seem to use different variables for convergence:
    Here, 
        F = error function AKA operator. Solve for F = 0
        b = norm of the error operator function, F, aka "eta" in H T Kung 4_2 algorithm. ALTERNATIVELY b is the newton step in Kantorovich.
        beta = norm of the jacobian inverse 
        L = the global lipschitz constant of the function, defined as the greatest spectral norm over the space

    return (h < 1/2 ? 1: 0)

    '''

    # make sure F and J are the right types
    F = np.array(F, dtype=float).flatten()
    J = np.array(J, dtype=float)

    print("Jacobian:", J)

    B = np.linalg.pinv(J) #B is J inverse

    if beta_preset > 0:
        beta = beta_preset
    elif beta_preset <= 0:
        beta = np.linalg.norm(B, ord=2)
    if beta_preset<0:
        beta = np.log10(beta)

    b = np.linalg.norm(F)

    if L == None:
        L = np.linalg.norm(J, ord=2) #apprxomiate L as the spectral norm of J

    print("beta", beta)
    print("L", L)
    print("b", b)
    h = beta * L * b 
    print("h", h)

    return (True if h <= 0.5 else False)



def jac_policy(robot: dh.DenavitHartenberg_Cameras_Analytic, initQ, desP):
    '''
    Aug 11 2025

    Use error bounds to monitor how much our 
    error_function has real world desired points and projects them, takes currQ
    '''
    currQ = initQ
    tolerance = 1e-3
    maxiter=50

    traj = [currQ]
    errors =[]
    jac_update_iters =[]

    desPP = robot.projected_world_point(desP)
    print("LOG desPP", desPP)

    J = robot.central_differences_pp(currQ, desPP)  
    print("J:\n", J)
    
    delQ = [0.]*robot.dh_robot.dof


    robot.calc_lipschitz(2.0) #calculate the lipschitz constant

    jac_count=1

    epsilon = 1.0 #the amount of error permitted from the amount of distance moved from where we did the jacobian initial estimation 

    step=1

    n_consecutive_small_steps=0


    for i in range(maxiter):
        print("###################### i:", i)
        currError = robot.projected_errfn_eval(currQ, desPP) #desired-current

        if np.linalg.norm(currError) <= tolerance:
            break

        Jinv = np.linalg.pinv(J)

        print("norm of Jinv:", np.linalg.norm(Jinv))

        corrQ = (Jinv @ currError).flatten()
        print(f"currQ: {currQ}\ncorrQ: {corrQ}\ncurrError: {currError}")
        
        #### BROYDEN UPDATE:
        broyden_numerator = np.multiply(np.reshape(np.subtract(currError, J@corrQ),(-1,1)), corrQ)
        broyden_denominator = np.array(corrQ) @ np.array(corrQ)
        jacobian_learning_rate=1.0
        J = J + jacobian_learning_rate * (broyden_numerator/broyden_denominator)
        print("Updated Jacobian:\n", J)


        delQ+=corrQ
        print("DELQ:", delQ)
        jacobian_approximated_error = (robot.lipschitz / 2) * (np.linalg.norm(delQ))**2
        
        print("jac approximated err:", jacobian_approximated_error)
    
        # Add the jacobian policy below!
        numerator = np.sqrt(2 * epsilon / robot.lipschitz)
        denominator = np.linalg.norm(delQ)
        #numerator = np.linalg.norm(currError)
        #denominator = np.linalg.norm(J*corrQ)

        print("numerator", numerator)
        print("denominator", denominator)


        martin_trust_regions=0
        if martin_trust_regions:
            d_lower = 0.1
            d_upper = 0.7

            d = numerator/denominator 
            print("d:", d)

            alpha = 1.0  # initial trust region radius

            if np.linalg.norm(corrQ) > alpha:
                corrQ = corrQ * (alpha / np.linalg.norm(corrQ))  # clip step to radius

            # compute agreement ratio
            d = np.linalg.norm(currError) / np.linalg.norm(J @ corrQ)

            if d < d_lower:
                alpha = 0.5 * alpha
            elif d > d_upper:
                alpha = max(2*alpha, np.linalg.norm(corrQ))
            # else: alpha unchanged

            print("alpha:", alpha)

            updatethreshold = 0.2

            if step < updatethreshold:
                if n_consecutive_small_steps == 3:
                    print("UPDATE JACOBIAN")
                    n_consecutive_small_steps=0
                    delQ=[0.]*robot.dh_robot.dof
                    J = robot.central_differences_pp(currQ, desPP)  
                    print("J:\n", J)
                    jac_count+=1
                    jac_update_iters.append(i)

                n_consecutive_small_steps+=1
            


        taylor_truncation = 1
        if taylor_truncation:
            lowerboundstep, updatethreshold, upperboundstep = (0.1, .3,  1)
            #when alpha is greater than 1, we tend to oscillate. But the step cannot be so small that it is neglible.
            step = min(upperboundstep, numerator/denominator)
            if step < lowerboundstep:
                step = lowerboundstep
            print("STEP:", step)
            if step < updatethreshold:
                print("UPDATE JACOBIAN")
                n_consecutive_small_steps=0
                delQ=[0.]*robot.dh_robot.dof
                J = robot.central_differences_pp(currQ, desPP)  
                print("J:\n", J)
                jac_count+=1
                jac_update_iters.append(i)

        kantorovich = 0
        if kantorovich:
            # make sure F and J are the right types
            J = np.array(J, dtype=float)

            B = np.linalg.pinv(J) #B is J inverse

            beta = np.linalg.norm(B, ord=2)
    
            b = np.linalg.norm(currError.flatten())

            L=robot.lipschitz

            print("beta", beta)
            print("L", L)
            print("b", b)
            h = beta * L * b 
            print("h", h)         

            

            


        
        currQ = currQ - step*corrQ
        traj.append(currQ)
        errors.append(np.linalg.norm(currError))

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

jac_policy(vs, initQ, desP)




