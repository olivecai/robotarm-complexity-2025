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

# TODO make robot.eval_err_function()
# TODO make robot.cd_jacobian()

def mod_newton_method(robot: dh.DenavitHartenberg_Cameras_Analytic, initQ):
    '''
    error_function has real world desired points and projects them, takes currQ
    '''
    currQ = initQ
    tolerance = 1e-3
    maxiter=200

    J = robot.cd_jacobian(currQ)      
    
    alpha = 0.5 

    for i in range(maxiter):
        currError = robot.eval_err_function(currQ) #desired-current
        if np.linalg.norm(currError) <= tolerance:
            break

        Jinv = np.linalg.pinv(J)

        corrQ = Jinv @ currError

        numerator = 2 * alpha * np.linalg.norm(currError)
        denominator = Lipschitz * (np.linalg.norm(corrQ) ** 2)
        step = min(1, np.sqrt(numerator/denominator))




