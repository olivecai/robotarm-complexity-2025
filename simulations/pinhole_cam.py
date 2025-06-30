'''
Pinhole camera that looks at a flat x-y plane. This camera exists in 3D and can rotate, translate in XYZ.
Objective: given x,y, return x,y coords'''

import numpy as np
import math
from spatialmath import SE3
from spatialmath.base import e2h, h2e
import machinevisiontoolbox as mvtb
from machinevisiontoolbox import CentralCamera
import roboticstoolbox as rtb
TOLERANCE=1e-5

#take only points and return those points. we can connect them later
    
def fkin3D(e, theta):
    ''' NOTE: idxing x and y coords is for SE3 translation/rotation matrix, not 2D!!!'''
    T= e.eval(theta)
    x=T[0][3]; 
    y=T[1][3];
    z=T[2][3];
    return np.array([x,y,z]);

def init_camera(f, rho, px, py, imagesize) -> CentralCamera:
    return mvtb.CentralCamera(f=f, rho=rho, pp=(px,py), imagesize=imagesize)

def plotVSerror(initialQ, desiredP, camera: CentralCamera, robot: rtb.Robot):
    '''
    step 1: solve y*-y_k = J @ change_in_x
    step 2: solve for J
    step 3: update: x_k+1 = x_k + change_in_x
    '''
    #using initialQ, get initialP. both of these are really jus
    initialRP=fkin3D(robot.ets, initialQ) #there is real point, proj point, and only real theta we work with. 
    initialPP=camera.project_point(initialRP)
    x,y=initialPP

    ''' For uncalibrated VS, we don't need the interaction matrix(?) 
    X,Y,Z=realP
    x = X/Z; y=Y/Z
    projP = np.array([x,y])
    interactionmatrix = np.array([[-1/Z, 0, x/Z, x*y, -(1+x^2), y],
                                  [0, -1/Z, y/Z, 1+y^2, -x*y, -x]])
    '''
    
    #solve y*-y_k to get position error
    error=desiredP-initialPP

    #loop beginning, set curr as initials
    currQ=initialQ
    currPP=initialPP

    i=0
    maxiter=100
    while i<maxiter and error>TOLERANCE:
        currRP = fkin3D(robot.ets, currQ) #get the current real point based on theta
        currPP = camera.project_point(currRP) #get the current projected point according to camera details
        #atp we have error (ie change in y), current PP, current Q.
    

    #using change in y and change in x, get the jacobian--but wait when do we use the interaction matrix?

    #is the interactino matrix our initial guess and then after that we use broyden's method?





def main():
    f = 8*1e-3     # focal length in metres
    rho = 10*1e-6  # pixel side length in metres
    u0 = 500       # principal point, horizontal coordinate
    v0 = 500       # principal point, vertical coordinate

    desired = np.array([1,1])
    testy=np.array([1,2,3])

    camera=init_camera(f, rho, u0, v0, 1000)
    projection= camera.project_point(testy)
    print(projection)
    print(camera.K)

    #plotVSerror(desired, camera)
            
        
main()