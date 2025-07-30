'''
July 14 2025

Kantovorich Theorem can give us results on semilocal convergence.

There are FOUR CONDITIONS on operator F and starting point x0:
Let X, Y, be Banach spaces, C contained in X, F: C-->Y a continuous function cont differentiable on int(C). 

K1: for x0 in X, there exists inv([F'(x0)]) in the set of bounded linear operators L(Y,X) st the operator norm of this inverse jacobian is <= some constant BETA
K2: the operator norm of the inverse jacobian of F at (x0) @ F(x0) is <= some constant ETA
K3: There exists a constant Lipschitz >=0 st the operator norm of [F'(x)-F'(y)] is <= L*[norm of x-y] for any x,y, in the subset C in the Banach space X
K4: h = Lipschitz * BETA * ETA <= 1/2 and B(x0,R) st R = ETA * [2*(1-a0)]/[2-3a0], where if a0 is >0 but <1/2, then the Newton sequence has R-order of convergence of at least two, while if a0==1/2, the seuqence has at least 1-order of convergence,

If K4 is valid, then we have guarenteed that:
- a solution exists
- a solution is unique
- we know some bound on the rate of convergence to this solution

The first two conditions are on starting point x0.
The third is on operator F.
The fourth combines the first three.

So, some takeaways:
K1 tells us that the inverse Jacobian of the starting position must exist (aka J is nonsingular). So, if we are initialized (or wind up in) a singular position, the first priority is to get out of that singular position.
K2 tells us how near we are to a solution already: this is, after all, the size of the first step! And remember that F(x0) is the error function that should == 0. The higher b is, the farther we are from the solution.
K3 tells us we need Lipschitz continuity of the Jacobian: the Jacobian change must be bounded by some LINEAR constant. The higher L is, the more unpredictable and nonlinear our function likely is. This tells us how predictable the region is. 
    Papers that focus on the domains of parameters and increasing that region focus on tuning the Lipschitz constant, which is something we should focus on.
K4 is THE condition and is called the Kantorovich condition! It simply tells us whether or not we have guaranteed convergence.

Here is some information we need to gather:

- Error function that maps DOF variables into a 3D task space (However, we will have to work in the 2D task space eventually): F(t1,t2,...tDOF): R^DOF -> R^3
    We can easily grab this from the Denavit Hartenberg Parameters Matrix third column (the translation vector)
    Needs the forward kinematics/ DH parameters. EASY
- The global lipschitz constant L.
    This may be quite difficult to obtain over a large joint space. This is a bit of a problem in and of itself. L = sup((operator norm of F'(x)-F'(y))/(operator norm of (x-y))
    It is potentially a little silly that we scrapped the other 'compute/calibrate beforehand' ideas but must go ahead with this, so we should see if there is a way to predict (or if there exists a general lower bound we can find) for this Lipschitz constant.
    Hmmmm... I also wonder if it is possible to get the Lipschitz constant locally, if that'll do?
    Needs the Jacobian of the error function. HARD
- The constant b, st the operator norm of [F'(x0)^(-1) @ F(x0)] is <= b. 
    This should be easy to attain, since we simply need to: compute the central differences, invert, and mat mult by the curr error fn. b= the constant obtained from this Newton step calculation :-)
    Needs x0 and the Error Function F. EASY
    
So, we can gather that, since we need the GLOBAL lipschitz constant, that this is a very conservative, safe lower bound. 
This tells us though, that for function landscapes with very high Lipschitz values, the radius of convergence may end up being far too conservative!

So, there is certainly a lot of value in deciding not to even go NEAR singular positions, since it will likely enlargen the radius of convergence. HYPOTHESIS 1

We should get started with coding the first three values:
- Error Function F # NOTE we don't need to know the exact operator, we can just sample this function.
- Lipschitz constant L
- Initial error constant b

And in that order.

TODO for regions of parameter: "How to improve the domain of parameters for Newton's method", J.A.Esquerro, M.A. Hernandez-Veron
    This paper is very interesting and notes that for a fixed x0, replacing L in K3 with a local Lipschitz constant L0 will always satisfy the equivalence expression, and that L0 can be used to increase the convergence radius. 
    The paper posits that the optimum case of Argyros is when K*BETA*ETA <= 1/ (1+MU) where MU = L0/L. 
    From there, notice that the smaller MU is, the bigger our radius of convergence is.
    So, this gives us some happy conclusions that were indicated above... the local Lipschitz evaluation is indeed effective! And, unsurprisingly, the smaller the local Lipschitz value, the better.

NOTE on the global Lipschitz constant in the real robot system:
- We don't know F, and can't get the global constant without sampling over the joint space.
- If we overestimate the constant, it's SAFER, but the tradeoff is that the radius of convergence is much smaller.
PLAN: Start off with the approximated Lipschitz constant. Maybe we can make a mesh of local Lipschitz regions, so as our joints travel through the space, we take on a different Lipschitz constant. 
- We only have to calibrate once beforehand to obtain the lipschitz constant for each region of our mesh. In fact, what if we have a jacobian AND a local lipschitz constant for regions in the mesh?
- HOWEVER: What if we get a certain radius, move there, and
'''

# ret err fn F

# ret lipschitz const L by iterating over the joint space 

# ret init error const b

from scipy.optimize import minimize
import sympy as sp
import numpy as np
import matplotlib.pyplot as plt

import denavit_hartenberg as dh

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

print(robot.J)
'''
This is robot.J:
Matrix([[0.3*sin(t0)*sin(t1)*sin(t2) - 0.3*sin(t0)*cos(t1)*cos(t2) - 0.55*sin(t0)*cos(t1), -0.3*sin(t1)*cos(t0)*cos(t2) - 0.55*sin(t1)*cos(t0) - 0.3*sin(t2)*cos(t0)*cos(t1), -0.3*sin(t1)*cos(t0)*cos(t2) - 0.3*sin(t2)*cos(t0)*cos(t1)], [-0.3*sin(t1)*sin(t2)*cos(t0) + 0.3*cos(t0)*cos(t1)*cos(t2) + 0.55*cos(t0)*cos(t1), -0.3*sin(t0)*sin(t1)*cos(t2) - 0.55*sin(t0)*sin(t1) - 0.3*sin(t0)*sin(t2)*cos(t1), -0.3*sin(t0)*sin(t1)*cos(t2) - 0.3*sin(t0)*sin(t2)*cos(t1)], [0, -0.3*sin(t1)*sin(t2) + 0.3*cos(t1)*cos(t2) + 0.55*cos(t1), -0.3*sin(t1)*sin(t2) + 0.3*cos(t1)*cos(t2)]])
'''

fn = (sp.utilities.lambdify(jntspace[:robot.dof], robot.J, 'numpy'))

# Objective: negative spectral norm (so that minimize → maximum)
def lipschitz_objective(q):
    J_val = fn(*q)
    norm_val = np.linalg.norm(J_val, ord=2).copy()
    #print("J:\n", J_val)
    #print("q:", q, "→ norm:", norm_val)
    return -norm_val

def test(q):
    f = np.array([[q[0],0,0],
                  [0,q[1],0],
                  [0,0,q[2]]])
    fn = np.linalg.norm(f, ord=2)
    print("f:\n", f)
    print("q:", q, "→ norm:", fn)
    return - fn


def empirical_lipschitz(robot: dh.DenavitHartenbergAnalytic, desP):
    '''
    simple lipschitz algorithm.

    random sample sparsely over the entire space to get central differences
    from central differences jacobian we can get that local lipschitz
    global lipschitz = max of all local lipschitz
    '''
    global_lipschitz = 0 #init as minimum value
    min_spectral_norm = np.inf

    num_samples = 50
    random_joint_configs = [
        np.array([np.random.uniform(low, high) for (low, high) in bounds])
        for _ in range(num_samples)
    ]
    i=0
    condnums = []
    spectralnums = []
    xrange = np.array(list(range(num_samples)))

    for Q in random_joint_configs:
        i+=1
        # get central differences jacobian
        jac = robot.central_differences(Q, desP)
        inv = np.linalg.pinv(jac)
        
        spectralnorm = np.linalg.norm(jac, ord=2)
        invnorm = np.linalg.norm(inv, ord=2)

        cond = spectralnorm* invnorm

        if spectralnorm > global_lipschitz:
            lipschitzQ=Q
            global_lipschitz = spectralnorm

        if spectralnorm < min_spectral_norm:
            min_spectral_norm = spectralnorm
        '''
        print("condition number:", cond)
        print(i, Q)
        print(jac)
        print(spectralnorm)'''

        condnums.append(np.log(cond))
        spectralnums.append(spectralnorm)

    print("global lipschitz:", global_lipschitz)
    print("minimum spectral norm:", min_spectral_norm)

    #plt.scatter(xrange, condnums, c='blue')
    #plt.scatter(xrange, spectralnums, c='red')
    #plt.show()

    #robot.plot(lipschitzQ)

    return global_lipschitz

def h_VS_successfuliterationcount(lipschitz, alpha, desP):
    num_samples = 200 # Tune this for accuracy vs speed

    random_joint_configs = [
        np.array([np.random.uniform(low, high) for (low, high) in robot.jointlimits])
        for _ in range(num_samples)
    ]

    h_vals = []
    ik_vals = []
    cond_vals = []
    b_vals = []
    B_vals = []

    i=0
    for Q in random_joint_configs:
        i+=1
        print(i, "#######################################")
        print("Q:", Q, "position of Q:", robot.fkin_eval(*Q))

        variables = robot.jntvars[: robot.dof] + robot.cartvars 

        print("error:", robot.errfn_eval(*Q , *desP))

        h, p, b, B = robot.ret_kantovorich(lipschitz, Q, desP, alpha)
        iters = robot.const_jac_inv_kin(desP, Q.tolist())

        jac = robot.central_differences(Q, desP)
        inv = np.linalg.pinv(jac)
        spectralnorm = np.linalg.norm(jac, ord=2)
        invnorm = np.linalg.norm(inv, ord=2)
        cond = spectralnorm* invnorm
        
        h_vals.append(h)
        ik_vals.append(iters)
        cond_vals.append(cond)
        b_vals.append(b)
        B_vals.append(B)

        print("h:", h ,"ik_val:", iters, "cond:", cond)
    
     # Sort everything by h
    h_vals, ik_vals, cond_vals, b_vals, B_vals = (list(t) for t in zip(*sorted(zip(h_vals, ik_vals, cond_vals, b_vals, B_vals))))
    converged_flags = [iters != -1 for iters in ik_vals]  # True if succeeded

    # Separate successes and failures
    h_success = [h for h, c in zip(h_vals, converged_flags) if c]
    ik_success = [i for i, c in zip(ik_vals, converged_flags) if c]
    cond_success = [c for c, s in zip(cond_vals, converged_flags) if s]

    h_fail = [h for h, c in zip(h_vals, converged_flags) if not c]
    ik_fail = [i for i, c in zip(ik_vals, converged_flags) if not c]
    cond_fail = [c for c, s in zip(cond_vals, converged_flags) if not s]

    # Create interactive Plotly plot
    import plotly.graph_objs as go
    from plotly.offline import plot

    trace_h = go.Scatter(
        x=h_success, y=ik_success,
        mode='markers', name='h vs iterations (success)',
        marker=dict(color='blue')
    )

    trace_cond = go.Scatter(
        x=h_success, y=cond_success,
        mode='markers', name='h vs condition number (success)',
        marker=dict(color='red')
    )
    trace_fail = go.Scatter(
        x=h_fail, y=cond_fail,
        mode='markers', name='h vs condition number (failure)',
        marker=dict(color='black', symbol='x')
    )

    trace_b = go.Scatter(
        x=h_vals, y=b_vals,
        mode='markers', name='h vs b ',
        marker=dict(color='blue')
    )
    trace_B = go.Scatter(
        x=h_vals, y=B_vals,
        mode='markers', name='h vs B',
        marker=dict(color='red')
    )
    trace_i = go.Scatter(
        x=h_vals, y=ik_vals,
        mode='markers', name='h vs iterations',
        marker=dict(color='black')
    )

    layout = go.Layout(
        title='Convergence Radius vs. IK Iteration Count',
        xaxis=dict(title='h (convergence radius)'),
        yaxis=dict(title='Value'),
        hovermode='closest'
    )

    fig = go.Figure(data=[trace_b, trace_B, trace_i], layout=layout)
    plot(fig)

# Initial guess and bounds
q0 = np.array([0.1]*robot.dof) #whatever is the maximum 
bounds = [(0, np.pi)]*robot.dof
print("BOUNDS")
print(bounds)


res = minimize(lipschitz_objective, q0, bounds=bounds)
L_estimate = -res.fun

print("\n\nEstimated global Lipschitz constant:", L_estimate)
print("At joint config:", res.x)
        

#################
desQ = [np.pi/4] * robot.dof
desP = robot.fkin_eval(*desQ).flatten().tolist()
print("desQ:", desQ, "\ndesP:\n", desP)
initQ = [0.5] * robot.dof
    
alpha=1.0
# Choose a starting position and the error function to minimize...
# Let's start with DOF2 first...
robot.plot(desQ)
robot.plot(initQ)

lipschitz = empirical_lipschitz(robot, desP) 
h, p, b, B = robot.ret_kantovorich(initQ, desP, alpha,lipschitz)
print("radius of convergence ball:" , p)

print("constant jacobian newton method:", robot.const_jac_inv_kin(desP, initQ))

#h_VS_successfuliterationcount(lipschitz, alpha, desP)
# generate a bunch of initQ
# get h and iterations for successful const jac inv for a initQ and desP --> reorder h from least to greatest and reorder the iteration count the same


#robot.view_invkin_task(initQ, desP, p)
#robot.invkin()

print(robot.central_differences([0.]*robot.dof, [x,y,z]))

