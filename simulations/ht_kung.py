'''
July 18

This program should be used in conjunction with kantovorich.py and denavit_hartenberg.py.

This program takes a given initial point and tries to find a better point. 

ChatGPT generated code for univariable case:

import numpy as np

# Original function and its derivative
def f(x):
    return x**3 - x - 1

def df(x):
    return 3*x**2 - 1

# Setup constants
x0 = 0.0
f0 = f(x0)
df0 = df(x0)

eta0 = np.abs(f0)  # ||f(x0)||
beta = 1 / np.abs(df0)  # crude estimate of ||f'(x)^-1||
K = 10.0  # assume Lipschitz constant of f'
r = 1.0
delta = 0.1
h0 = beta**2 * K * eta0

print(f"Initial h0: {h0:.4f}, eta0: {eta0:.4f}, beta: {beta:.4f}")

# Build deformed function f_i and apply Newton on it
def step4_newton_on_fi(x0, f0, eta0, beta, K, r, delta, max_iters=20, tol=1e-8):
    h = beta**2 * K * eta0
    eta = eta0
    x_bar = x0
    
    for i in range(max_iters):
        print(f"\n--- Iteration {i} ---")
        print(f"h = {h:.4f}, eta = {eta:.4f}")
        
        if h <= 0.5 - delta:
            print(f"Stopping: h = {h} is small enough.")
            return x_bar
        
        lam = (0.5 - delta) / h
        
        # Define the deformed function f_i and its derivative
        def f_i(x):
            return (f(x) - eta * f0 / eta0) + lam * eta * f0 / eta0

        def df_i(x):
            return df(x)  # since f_i is just a linear combo of f, the derivative is still f'

        # Apply Newton's method on f_i
        x = x_bar
        for j in range(20):
            fx = f_i(x)
            dfx = df_i(x)
            step = fx / dfx
            x_new = x - step

            # Check the Step 4 conditions
            cond1 = np.abs(x_new - x) <= r - 2 * beta * eta0
            cond2 = np.abs(f_i(x_new) / df_i(x_new)) <= min(r/2 - beta * eta, delta / (2 * beta * K))

            print(f"  Newton iter {j}: x = {x_new:.6f}, |step| = {np.abs(step):.2e}, cond1 = {cond1}, cond2 = {cond2}")
            
            if cond1 and cond2:
                print("  ✅ Step 4 conditions satisfied. Accepting x_new.")
                x_bar = x_new
                break
            x = x_new

        # Update eta and h for next round
        eta = (1 - lam) * eta
        h = beta**2 * K * eta

    return x_bar

# Run it
x_good = step4_newton_on_fi(x0, f0, eta0, beta, K, r, delta)
print(f"\nFinal good starting point: x = {x_good:.6f}")

#### SAMPLE OUTPUT ####

Initial h0: 2.5000, eta0: 1.0000, beta: 2.0000

--- Iteration 0 ---
h = 2.5000, eta = 1.0000
  Newton iter 0: x = 0.666667, |step| = 6.67e-01, cond1 = False, cond2 = False
  Newton iter 1: x = 0.858889, |step| = 1.92e-01, cond1 = False, cond2 = True
  Newton iter 2: x = 1.018424, |step| = 1.60e-01, cond1 = True, cond2 = True
  ✅ Step 4 conditions satisfied. Accepting x_new.

--- Iteration 1 ---
h = 0.6250, eta = 0.2500
...
Final good starting point: x = 1.313579


### MULTIVARIABLE CASE ### 

def fi(x, f0, eta0, etai, lambdai):
    const_shift = (lambdai - 1) * etai * f0 / eta0
    return f(x) + const_shift

def newton_step_fi(x_init, f0, eta0, etai, lambdai, r, beta, K, delta, max_iters=20):
    x = x_init.copy()

    for i in range(max_iters):
        fx = fi(x, f0, eta0, etai, lambdai)
        Jx = J(x)  # Reuse original Jacobian
        try:
            dx = np.linalg.solve(Jx, fx)
        except np.linalg.LinAlgError:
            print("Jacobian singular!")
            return None

        x_new = x - dx

        # Check Algorithm 4.2 Step 4 conditions:
        cond1 = np.linalg.norm(x_new - x) <= r - 2 * beta * eta0
        cond2 = np.linalg.norm(np.linalg.solve(Jx, fi(x_new, f0, eta0, etai, lambdai))) <= min(
            r/2 - beta * etai,
            delta / (2 * beta * K)
        )

        print(f"Iter {i}: x = {x_new}, |dx| = {np.linalg.norm(dx):.2e}, cond1 = {cond1}, cond2 = {cond2}")

        if cond1 and cond2:
            print("✅ Both conditions satisfied. Returning new x.")
            return x_new

        x = x_new

    print("❌ Did not converge within Newton iterations.")
    return None

'''