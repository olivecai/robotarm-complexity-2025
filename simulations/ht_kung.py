'''
July 28 2025

This program is simply used for testing code.
'''
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
robot = dof2

cam1 = dh.Camera(0,0,0,[0,0,5], 5,5, 0, 0) #looks down directly at the scene, basically replicates the scene
cam2 = dh.Camera(-sp.pi/2, 0, 0, [0,0,5], 5,5,0,0) #looks at scene from the y axis, world z is cam2 y, world x is cam2 x 
cameras=[cam1, cam2]

vs = dh.DenavitHartenberg_Cameras_Analytic(cameras, robot)

initQ = [0, 2.]
desP = [1,1,0]

# Original function and its derivative
def f(x: list):
    return vs.errfn_eval(*x).copy()

def df(x: list):
    return vs.jacobian_eval(*x).copy()

# Setup constants
vars0 = [*initQ, *desP]
x0 = initQ
f0 = f(vars0).copy()
df0 = df(vars0)


eta0 = np.linalg.norm(f0)  # ||f(x0)||
beta =np.linalg.norm(df0, ord=2)  # crude estimate of ||f'(x)^-1||
K = vs.calc_lipschitz()  # assume Lipschitz constant of f'
r = 1.0
delta = 0.1
h0 = beta**2 * K * eta0

print(f"Initial h0: {h0:.4f}, eta0: {eta0:.4f}, beta: {beta:.4f}")

print(f0)

print(df0)
# Build deformed function f_i and apply Newton on it
def step4_newton_on_fi(x0, f0, eta0, beta, K, r, delta, max_iters=20, tol=1e-8):
    h = beta**2 * K * eta0
    eta = eta0
    x_bar = x0
    vars = [*x_bar, *desP]
    
    for i in range(max_iters):
        print(f"\n--- Iteration {i} ---")
        print(f"h = {h:.4f}, eta = {eta:.4f}")
        
        if h <= 0.5 - delta:
            print(f"Stopping: h = {h} is small enough.")
            return x_bar
        
        lam = (0.5 - delta) / h
        
        # Define the deformed function f_i and its derivative
        def f_i(x: list):
            fx = f(x).flatten()
            f0_flat = f0.flatten()
            ret = (fx - eta * f0_flat / eta0) + lam * eta * f0_flat / eta0
            return ret

        def df_i(x: list):
            return df(x)  # since f_i is just a linear combo of f, the derivative is still f'

        eta = (1-lam)* eta

        # Apply Newton's method on f_i
        x = x_bar
        
        for j in range(20):
            vars = [*x, *desP]
            fx = f_i(vars)
            dfx = df_i(vars)
            step = np.linalg.pinv(dfx) @ fx # this is analagous to Jac inverse @ error fn 
            x_new = x - step 

            # Check the Step 4 conditions
            cond1 = np.linalg.norm(x_new - x) <= r - 2 * beta * eta0
            vars= [*x_new, *desP]

            cond2 = np.linalg.norm(np.linalg.pinv(df_i(vars)) @ f_i(vars)) <= min(r/2 - beta * eta, delta / (2 * beta * K))

            print(f"  Newton iter {j}: x = {x_new}, |step| = {np.linalg.norm(step):.2e}, cond1 = {cond1}, cond2 = {cond2}")

            if cond1 and cond2:
                print("  ✅ Step 4 conditions satisfied. Accepting x_new.")
                x_bar = x_new
                break
            x = x_new
            vars = [*x, *desP]

        h = beta**2 * K * eta 

    return x_bar

# Run it
x_good = step4_newton_on_fi(x0, f0, eta0, beta, K, r, delta)
print(f"\nFinal good starting point: x = {x_good}")
