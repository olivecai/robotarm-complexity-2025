'''
May 30 2025

Some common functions like forward kinematics and jacobian calculations are very common, 
so it would be nice to store them here.

Here we can compile many of the functions we have been using up till now. (Cleaning up a little.)
'''

import numpy as np
from spatialmath import SE3
from spatialmath.base import e2h, h2e
import machinevisiontoolbox as mvtb
import roboticstoolbox as rtb

PI = np.pi

def fkin(q, ets):
    T = ets.eval(q)
    pos = T[:3, 3]  # safer and clearer
    return pos

def proj_point(realP, camera: mvtb.CentralCamera):
    '''
    This function only exists because I didn't like the format of the 
    vision toolbox point projection, which is [[a],[b]] and I want [a,b]
      
    Takes in a real world point in SE3 and returns the projection in the x-y plane.
    '''
    realP = h2e(np.linalg.inv(camera.T.A) @ e2h(realP))
    projP = camera.project_point(realP)
    posPx = projP[0][0]
    posPy=projP[1][0]
    return np.array([posPx, posPy])

def vs_fkin(e,theta, camera: mvtb.CentralCamera):
    '''
    returns the position of the projection of the EE.  
    '''
    posR = fkin(theta, e)
    #print("Real point: ", posR)
    posP = proj_point(posR, camera)
    #print("POSP:", posP)
    return posP

def init_camera(f, rho, px, py, imagesize) -> mvtb.CentralCamera:
    camera = mvtb.CentralCamera(f=f, rho=rho, pp=(px,py), imagesize=imagesize)
    camera.T = SE3(0,0,-5)
    return camera

def vs_centraldiff_jacobian(currQ, e: rtb.Robot.ets, camera: mvtb.CentralCamera):
    '''
    Visual Servoing;
    Initialize and return a Jacobian through central differences, without use of a camera.'''
    epsilon=1e-5
     ###sanity check for our dimensions: 
    # let p=robot params, k=datapts for overdet system. 
    # each dP=1xd, each dQ=1xp. after generating overdet sys, 
    # DP=kxd, DQ=kxp, currJ.T=pxd... so currJ=dxp. 
    # Also, I is pxp
    # in this case, p=3, d=2
    p=currQ.shape[0]
    d=vs_fkin(e, currQ, camera).shape[0]
 #d is world dimensions
    k=1 #for now its not overdetermined. NOTE: find out if Dylan's paper used overdet.
    currJT = np.zeros((p,d)) 
    I=np.identity(p)
    for i in range(p):
        forward = vs_fkin(e, currQ + epsilon * I[i], camera)
        backward = vs_fkin(e, currQ - epsilon * I[i], camera)
        currJT[i] = (forward - backward) / (2 * epsilon)

    return currJT.T

def centraldiff_jacobian(currQ, e: rtb.Robot.ets):
    '''
    Initialize and return a Jacobian through central differences, without use of a camera.'''
    epsilon=1e-5
     ###sanity check for our dimensions: 
    # let p=robot params, k=datapts for overdet system. 
    # each dP=1xd, each dQ=1xp. after generating overdet sys, 
    # DP=kxd, DQ=kxp, currJ.T=pxd... so currJ=dxp. 
    # Also, I is pxp
    # in this case, p=3, d=2
    p=currQ.shape[0]
    d=fkin(currQ, e).shape[0]
 #d is world dimensions
    k=1 #for now its not overdetermined. NOTE: find out if Dylan's paper used overdet.
    currJT = np.zeros((p,d)) 
    I=np.identity(p)
    for i in range(p):
        forward = fkin(currQ + epsilon * I[i], e)
        backward = fkin(currQ - epsilon * I[i], e)
        currJT[i] = (forward - backward) / (2 * epsilon)

    return currJT.T

def analytic_jacobian(q: np.ndarray, ets: rtb.ETS) -> np.ndarray:
    '''
    Computes the analytic Jacobian of a robot defined by an ETS at specific configuration q.

    Returns:
        np.ndarray: The Jacobian matrix (6 x N), where N is the number of joints.
    '''
    robot = rtb.ERobot(ets)
    return robot.jacob0(q)[:3, :]

