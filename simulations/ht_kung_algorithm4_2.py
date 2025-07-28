'''
July 27 2025

PERFORM INVERSE KINEMATICS: Check if kantorovich conditions are satisfied. If not, find a better starting point, then continue to attempt to find a better starting point.
This program implements Algorithm 4.2 from H.T.Kung's paper on finding better starting points when Kantorovich conditions fail for an initial starting point.

Uses object classes from denavit_hartenberg to obtain analytic equations for F and the lipschitz constant.

DELIVERABLES:
- How many iterations does it take to solve an inverse kinematics problem using this algorithm?
- What is the lipschitz constant of the system with the camera VS without the camera?
'''
from denavit_hartenberg import DenavitHartenberg_Cameras_Analytic, DenavitHartenbergAnalytic, Camera, DHSympyParams
import numpy as np

def alg_4_2(system: DenavitHartenberg_Cameras_Analytic, initQ, desP):
    x0 = initQ
    f0 = system.errfn_eval(*x0) #evaluate the initial function
    J0 = system.central_differences(*x0)
    B0 = np.linalg.pinv(J0)

    beta = np.linalg.norm(B0, ord=2) #spectral norm of inverse jacobian
    K = system.lipschitz