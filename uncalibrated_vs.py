'''
Uncalibrated visual servoing. 
1. Setup: camera and arm in ready pose exist in R3. Use the real points for desired and EE initially to project them into the camera. After that, we will only access the camera.
2. Initialization: Let errorP = currP-desiredP
3. Use an overdetermined system and central differences to estimate the Jacobian.
4. After initializing the Jacobian, use the Jacobian and errorP to estimate the best change to currQ. (pinv(J)@errorP = dQ)
5. Update theta values: currQ=currQ+dQ
6. Update Jacobian using Broyden's method: currJ = currJ + ((dP.T - currJ*dQ)*dQ.T)/(dQ.T*dQ)
7. Each iteration, check if errorP is less than tolerance.
'''

import numpy as np
import math
from spatialmath import SE3
from spatialmath.base import e2h, h2e
import machinevisiontoolbox as mvtb
from machinevisiontoolbox import CentralCamera
import roboticstoolbox as rtb
import matplotlib.pyplot as plt
PI=np.pi
TOLERANCE=1e-5
MAXITER=50;
#ROBOT PARAMS
l0=1
l1=1
QR0=PI/2
QR1=-PI/2

def fkin3D(e, theta):
    ''' NOTE: idxing x and y coords is for SE3 translation/rotation matrix, not 2D!!!'''
    T= e.eval(theta)
    x=T[0][3]; 
    y=T[1][3];
    #z=T[2][3]; #for now to avoid div by zero (THOUGH WE NEED TO FIGURE THAT OUT)
    z=3;
    return np.array([x,y,z]);

def proj_point(realP, camera: CentralCamera):
    '''
    This function only exists because I didn't like the format of the 
    vision toolbox point projection, which is [[a],[b]] and I want [a,b]
      
    Takes in a real world point in SE3 and returns the projection in the x-y plane.
    '''
    projP = camera.project_point(realP)
    posPx = projP[0][0]
    posPy=projP[1][0]
    return np.array([posPx, posPy])
    

def vs_fkin(e,theta, camera: CentralCamera):
    '''
    returns the position of the projection of the EE.  
    '''
    posR = fkin3D(e,theta)
    #print("Real point: ", posR)
    posP = camera.project_point(posR)
    posPx = posP[0][0]
    posPy = posP[1][0]
    return np.array([posPx, posPy])

def init_camera(f, rho, px, py, imagesize) -> CentralCamera:
    return mvtb.CentralCamera(f=f, rho=rho, pp=(px,py), imagesize=imagesize)

def uncalibrated_vs(currQ, desiredP, camera: CentralCamera, e: rtb.Robot.ets):
    '''
    1. Setup: camera and arm in ready pose exist in R3. Use the real points for desired and EE initially to project them into the camera. After that, we will only access the camera.
    2. Initialization: Let errorP = currP-desiredP
    3. Use an overdetermined system and central differences to estimate the Jacobian.
    4. After initializing the Jacobian, use the Jacobian and errorP to estimate the best change to currQ. (pinv(J)@errorP = dQ)
    5. Update theta values: currQ=currQ+dQ
    6. Update Jacobian using Broyden's method: currJ = currJ + ((dP.T - currJ*dQ)*dQ.T)/(dQ.T*dQ)
    7. Each iteration, check if errorP is less than tolerance.
    '''
    i=0; 
    currP=vs_fkin(e, currQ, camera) #projection of that real point through the camera
    errorP=desiredP-currP
    error=np.linalg.norm(errorP)
    if error < TOLERANCE:
        return (i, error, errorP[0], errorP[1], currP, currQ) # our ready pose IS the desired pose

    epsilon=math.radians(0.15)
    alpha=0.5

    ###sanity check for our dimensions: 
    # let p=robot params, k=datapts for overdet system. 
    # each dP=1xd, each dQ=1xp. after generating overdet sys, 
    # DP=kxd, DQ=kxp, currJ.T=pxd... so currJ=dxp. 
    # Also, I is pxp
    # in this case, p=2, d=2
    p=currQ.shape[0]
    d=2 #d is world dimensions
    k=1 #for now its not overdetermined. NOTE: find out if Dylan's paper used overdet.
    currJT = np.zeros((p,d)) 
    DP=np.zeros((k,d))
    DQ=np.zeros((k,p))
    I=np.identity(p)
    for i in range(p): #i is a specific robot param
        #the jacobian is first initialized with central differences. 
        '''print("currQ: ")
        print(currQ)
        print(I[i])
        print(vs_fkin(e, currQ+epsilon*I[i], camera))'''

        currJT[i] = ((vs_fkin(e, currQ+epsilon*I[i], camera))-(vs_fkin(e, currQ-epsilon*I[i], camera)))/(2*epsilon)
    currJ=currJT.T
    # NOTE: advised to make f: actual theta -> result image position. double check if mine is OK! :>
    ###now that we have a Jacobian estimate, we can solve for the correction amt in Q.
    corrQ = np.linalg.pinv(currJ) @ errorP
    currQ=currQ+corrQ
    ###after solvign for the new theta, we move there and check the error.
    currP=vs_fkin(e, currQ, camera)
    errorP=desiredP-currP #error is measured from projection img
    error=np.linalg.norm(errorP)
    i+=1

    ### after the initial setup and estimating J, we begin the iterations: 
    while i< MAXITER and error>TOLERANCE:
        #update J using Broyden's method:
        currJ = currJ + alpha* ((errorP.T - currJ@corrQ)@corrQ.T)/(corrQ.T@corrQ)

        corrQ = np.linalg.pinv(currJ) @ errorP #calculate the change in theta needed
        currQ=currQ+corrQ #update the current theta 
        #use the current theta to find our new position and error:
        currP=vs_fkin(e, currQ, camera)
        errorP=desiredP-currP
        Xerror=errorP[0]
        Yerror=errorP[1]
        error=np.linalg.norm(errorP)
        i+=1;
    ### return i, error, pos
    return i, error, Xerror, Yerror, currP, currQ      

def plot_error(e, desiredPP, tolerance, camera, mode):
    '''
    This function plots total error, x error, y error.
    Create a linear space of values of theta between -pi and pi,
    and for each pair of theta values, compute the jacobian and try to converge to desired pos. 
    Then, plot the error for each pair of theta values.
    '''
    # make a linspace and meshgrid for ranges of q0 and q1
    n = 100  #resolution
    q_range = np.linspace(-PI, PI, n )
    Q0, Q1 = np.meshgrid(q_range, q_range) 
    TotalError = np.zeros_like(Q0)
    XError = np.zeros_like(Q0)
    YError = np.zeros_like(Q0)
    # double loop thru q0 and q1, compute inverse kinematics at that starting point and compute the error
    for i in range(n):
        for j in range(n):
            q0 = Q0[i, j]
            q1 = Q1[i, j]
            if mode==1:
                iterations, TotalError[i,j], XError[i,j], YError[i,j], position, theta = uncalibrated_vs(np.array([q0,q1]), desiredPP, camera, e) 

    #Plot the Error Surface 
    fig = plt.figure(figsize=(12, 4))

    # First plot: total error
    ax1 = fig.add_subplot(1, 3, 1, projection='3d')
    ax1.plot_surface(Q0, Q1, TotalError, cmap='plasma')
    ax1.set_xlabel("q0 (rad)")
    ax1.set_ylabel("q1 (rad)")
    ax1.set_zlabel("Total Position Error")
    ax1.set_title("Total Error Surface")

    # Second plot: x error
    ax2 = fig.add_subplot(1, 3, 2, projection='3d')
    ax2.plot_surface(Q0, Q1, XError, cmap='plasma')
    ax2.set_xlabel("q0 (rad)")
    ax2.set_ylabel("q1 (rad)")
    ax2.set_zlabel("X Position Error")
    ax2.set_title("X Error Surface")

    # Third plot: y error
    ax3 = fig.add_subplot(1, 3, 3, projection='3d')
    ax3.plot_surface(Q0, Q1, YError, cmap='plasma')
    ax3.set_xlabel("q0 (rad)")
    ax3.set_ylabel("q1 (rad)")
    ax3.set_zlabel("Y Position Error")
    ax3.set_title("Y Error Surface")

    if mode==1:
        plt.suptitle("Mode 1: Measure error after visual servoing from q0,q1 for desired pos of 1,1,3. Constant depth")
    if mode==2:
        plt.suptitle("Mode 2: Measure error directly from q0,q1")
    plt.tight_layout()
    plt.show()

def main():
    #CAMERA PARAMS
    f = 8*1e-3     # focal length in metres
    rho = 10*1e-6  # pixel side length in metres
    u0 = 500       # principal point, horizontal coordinate
    v0 = 500       # principal point, vertical coordinate

    teeny=1e-1

    camera : CentralCamera = init_camera(f, rho, u0, v0, 1000)

    initialQ = np.array([QR0+teeny, QR1+teeny]) #ready position of the robot
    desiredRP = np.array([1,1,3])
    desiredPP = proj_point(desiredRP, camera)

    #just use ets. but if we want to use a robot, we can simply transform the robot into an ets
    ets = rtb.ET.Rz() * rtb.ET.tx(l0) * rtb.ET.Rz() * rtb.ET.tx(l1) #each arm is 1m long.

    iterations, error, Xerror, Yerror, position, theta = uncalibrated_vs(initialQ, desiredPP, camera, ets)
    print("iterations:", iterations,"error:", error, "resulting projected position:", position)
    print("real pos of the resultant projection:",fkin3D(ets, theta))
    print("desired real pos:", desiredRP, "desired projected pos:", desiredPP)

    print("Creating plot of q0 vs q1 vs position error, for visual servoing.")
    plot_error(ets, desiredPP, TOLERANCE, camera, 1)

main()

