'''
July 29 2025

This method should:
- use as few jacobians as possible
- use tools from semiconvergence to assess the 'goodness' of our current position
- deal with singularities

From starting position:



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

print("ROBOT:\n",robot.J)

cam1 = dh.Camera(0,0,0,[0,0,2], 2,2, 0, 0) #looks down directly at the scene, basically replicates the scene
cam2 = dh.Camera(-sp.pi/2, 0, 0, [0,0,2], 2,2,0,0) #looks at scene from the y axis, world z is cam2 y, world x is cam2 x 
cameras=[cam1, cam2]

vs = dh.DenavitHartenberg_Cameras_Analytic(cameras, robot)
vs.dh_robot.alpha = 0.5

kinova_angles = np.deg2rad(np.array([-0.1336059570312672, -28.57940673828129, -179.4915313720703, -147.7, 0.06742369383573531, -57.420898437500036, 89.88030242919922, 0.5]))
kinova_end = np.deg2rad(np.array([25.336059570312672, 50.57940673828129, -179.4915313720703, -90.7, 30.06742369383573531, -57.420898437500036, 30.88030242919922, 0.5]))

# initialize init Q and de
# initialize init Q and des P

initQ = [0.5] * robot.dof
desQ = [1.5] * robot.dof

desP =  (vs.dh_robot.fkin_eval(*desQ)).flatten().tolist()
print("desired world point:", desP)
print("initial world point:", vs.dh_robot.fkin_eval(*initQ).flatten().tolist())

# what is current projected point of end effector and desP in each camera?
for i in range(len(cameras)):
    print(f"\n#####################################\nProjected desired point in camera{i}:")
    print(cameras[i].projectpoint(desP))    

    print(f"\nProjected end effector position point in camera{i}:")
    print(cameras[i].projectpoint(vs.dh_robot.fkin_eval(*initQ)))

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


def method_regionsearch(vs: dh.DenavitHartenberg_Cameras_Analytic, initQ: list, desP: list):
    '''
    JACOBIAN: For every milestone, calculate the camera projection jacobian based on the fkin function, NOT the error function.

    We need to avoid colliding with ourself, which can be difficult to do if we simply follow the straight line between point A to B.
    It needs to be decided what scenarios are acceptable to not discuss... (ie singularity/collision avoidance, broyden updates, etc)

    If we do a general region search, there are two main differences:
    - The challenge of choosing a point that can exist as a valid constraint in cameras 1.. n concurrently. But if we use the technique of only having a few iterations per jacobian, we do not need to worry so much about actually attaining a goal... it's like we have an initial hunt. Maybe we could cap the number of jacobian updates in the beginning. Maybe the farther we are, the fewer iterations we allow
    - At least right now, I am not sure if there is an obvious search method we should implement.

    Emphasizing that there should be some dynamic change in the number of allowed iterations per Jacobian could be considered a little cheat. Should we allow it? It can be argued it is part of our 'searching for better points' phase, much like in H.T.Kung's algorithm; the iterations needed to deform the function successfully is very very little (less than 5 or 10)
    
    The farther we are, the shorter the Jacobian is valid for. Which makes sense. So can the number of iterations per Jacobian be a function of b, the norm of the error fucntion?
    In general we don't really ever need more than 300 iterations for an entire trajectory as seen empirically, 
    This also brings us back to Dylan's paper, which showed that there is a conglomerate region in task space where 1 jacobian is needed.
    And this brings us back to the 'minimum spanning set of jacobian' experiments from last month, where the region with 1 jacobian was very large, 2 jacobians was smaller, 3 was vastly smaller, so on so forth, until at the poles of the singularities, there was simply 1 or 2 points that needed 12 jacobian updates or so; the farther we are, the more jacobians needed increases nonlinearly.
    
    The motivattion of this function is almost to PROLONG the lifespan of a Jacobian. 
    '''
    # we can work on mesh refinment later with this method, but for now let's just sample most of the pointsin the genrated rectangle.
    '''
    PSUEDOCODE
    initialize. currP=initP, etc.
    jacobians needed = 0
    while current position is not at the desired position: (this makes it okay if the milestone chosen is essentially at the desired position, because this line will terminate any more iterations, so there is no 'double counting')
        generate a rectangle around currP and desP
        for each point in rectangle, evaluate the Kantorovich conditions:
            > return the point closest to the desired position, plus its kantorovihc conds.
        select a number of maxiters and alpha as a function of those kantorovich conditiions:
            > maxiter_i = 200 (or some number 100-300) / max(1, b_i) where b_i is the norm of the errorr fucntion: simply how much errro there currently is. 
            > alpha_i = 
        run newton's method for the number of maxiters:
            > currP becomes the point that we left off at. 
            > (using a small number of iterations if we are far away is effective because it is highly unlikely that the points we have chosen as our milestone can actually satisfy a realistic constraint, so the closer we get, the more feasible that milestone point actually is....)
        return the totla number of jacobian updates needed.
        
    '''


def method_binarysearch(vs: dh.DenavitHartenberg_Cameras_Analytic, initQ: list, desP: list):
    '''
    Get Kantorovich conditions.
    If not satisfied, then search for better points.
    '''

    eeP =vs.projected_world_point(vs.dh_robot.fkin_eval(*initQ))
    desPP = vs.projected_world_point(desP)
    J = vs.central_differences_pp(initQ, desPP)

    def close_enough(arr1, arr2, tolerance=1e-2):
        '''
        returns true if arr1 and arr2 are essentially the same vector.
        '''
        ret = 1
        for entry in np.abs(np.subtract(arr1, arr2)):
            if entry > 1e-1:
                ret = 0
        return ret

    def method_binarysearch_helper(furthest, start, end):
        '''
        recursive binary search for better points.
        Given two points, find middle point
        '''
        print("binary search method!")
        print("furthest:", furthest, "start:", start, "end:", end)
        middle = np.add(end, start) / 2 #get the vector in between
        
        basecase_ret = 1
        for entry in np.abs(np.subtract(end, start)):
            if entry > 1e-3:
                basecase_ret = 0 #if any of the entries have values greater than some_tolerance, shuo ming start he end hai shi you qu bie
        if basecase_ret:
            print("Basecase, returning...", furthest)
            return furthest

        F = np.subtract(vs.projected_world_point(vs.dh_robot.fkin_eval(*currentQ)), middle)

        semilocal_convergence = eval_kantorovich_conditions(F, J, beta_preset=-1, L=None)
        print("middle:", middle)
        print("Middle is Good:", semilocal_convergence)
        if semilocal_convergence: #a successful point has been found. continue the bianry searcht o find an even farther poitn. 
            return method_binarysearch_helper(furthest = middle, start=middle, end=end)
        else: #the middle was not a successful point. search in the half closer to the start.
            return method_binarysearch_helper(furthest=furthest, start=start, end=middle)

    milestone = eeP
    currentP = milestone
    print("END EFFECTOR INITIAL POSITION", eeP)
    milestones = []
    milestones.append(milestone)

    currentQ = initQ
    vs.dh_robot.plot(initQ)
    vs.dh_robot.plot(desQ)

    while 1: 
        milestone = method_binarysearch_helper(furthest=currentP, start=currentP, end=desPP) #current Position changes based on how successful the mini newton loop is in getting to the milestone
        milestones.append(milestone)
        print("START CHORD METHOD FROM:", currentP)
        print("TO THE MILESTONE:", milestone)
        print("Current Q:", currentQ)
        iters, currentQ = vs.const_jac_inv_kin_pp(milestone, currentQ, J)
        currentP= vs.projected_world_point(vs.dh_robot.fkin_eval(*currentQ))
        J = vs.central_differences_pp(currentQ, desPP)
        print("i:", iters)
        print("currentQ:", currentQ)
        print("currentP:", currentP)
        print("desPP:", desPP)
        print("milestone:", milestone)
        if close_enough(desPP, vs.projected_world_point(vs.dh_robot.fkin_eval(*currentQ)), 1e-1):
            break

    while 0:
        print("CALCULATING MILESTONE")
        prev_milestone = milestone.copy()
        milestone = method_binarysearch_helper(furthest=milestone, start=milestone, end=desPP)
        milestones.append(milestone)
        print("START FROM", prev_milestone)
        print("AND GET HERE:", milestone)
        print("milestone Q to start from:", milestoneQ)
        iters, milestoneQ = vs.const_jac_inv_kin_pp(milestone, milestoneQ, J)
        J= vs.central_differences_pp(milestoneQ, desPP)
        print("i:", iters)
        print("milestoneQ:", milestoneQ)
        print("currentP:", vs.projected_world_point(vs.dh_robot.fkin_eval(*milestoneQ)))
        print("desPP:", desPP)
        print("milestone:", milestone)
        if close_enough(desPP, vs.projected_world_point(vs.dh_robot.fkin_eval(*milestoneQ)), 1e-1):
            break


    print("FINSIHED. MILESTONES:\n", np.array(milestones), "fin")

    

# method: if Kantorovich conditions are not satisfied, then binary search desP closer
# when we find the furthest desP in the line, maybe we can find more nearby, but leave this later. 

#plot_projected_scene(cameras=cameras, eeP=vs.dh_robot.fkin_eval(*initQ), desP=desP)
print(initQ)
print(desP)


print(vs.errfn_eval(*initQ, *vs.cartvars))
method_binarysearch(vs, initQ, desP)


'''
desPP = vs.projected_world_point(desP.copy())
print("desP:", desP)
print("desPP:", desPP)
print("cd", vs.central_differences(initQ, desP))
print("cd pp", vs.central_differences_pp(initQ, desPP))'''
