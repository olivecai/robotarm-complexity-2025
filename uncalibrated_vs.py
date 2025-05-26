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
import plotly.express as px
import plotly.graph_objects as go

PI=np.pi
TOLERANCE=1e-5
MAXITER=10;
#ROBOT PARAMS
l0=1
l1=1
l2=1
QR0=PI/2
QR1=PI/2
QR2=PI/2

#CAMERA PARAMS
f = 8*1e-3     # focal length in metres
rho = 10*1e-6  # pixel side length in metres
u0 = 500       # principal point, horizontal coordinate
v0 = 500       # principal point, vertical coordinate

teeny=1e-6
initialQ = np.array([QR0+teeny, QR1+teeny, QR2+teeny]) #ready position of the robot

desiredRP = np.array([1,0,1])

def fkin3D(e, theta):
    ''' NOTE: idxing x and y coords is for SE3 translation/rotation matrix, not 2D!!!
    End effector position IRL
    '''
    T= e.eval(theta)
    x=T[0][3]; 
    y=T[1][3];
    z=T[2][3];
    return np.array([x,y,z]);

def proj_point(realP, camera: CentralCamera):
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
    

def vs_fkin(e,theta, camera: CentralCamera):
    '''
    returns the position of the projection of the EE.  
    '''
    posR = fkin3D(e,theta)
    #print("Real point: ", posR)
    posP = proj_point(posR, camera)
    #print("POSP:", posP)
    return posP

def init_camera(f, rho, px, py, imagesize) -> CentralCamera:
    camera = mvtb.CentralCamera(f=f, rho=rho, pp=(px,py), imagesize=imagesize)
    #camera.T = SE3(0, 0, 10) * SE3.Rx(np.pi)
    camera.T = SE3(0,0,-5)
    return camera

def centraldiff_jacobian(currQ, desiredP, camera: CentralCamera, e: rtb.Robot.ets):
    '''
    Initialize and return a Jacobian through central differences.'''
    epsilon=PI/16
     ###sanity check for our dimensions: 
    # let p=robot params, k=datapts for overdet system. 
    # each dP=1xd, each dQ=1xp. after generating overdet sys, 
    # DP=kxd, DQ=kxp, currJ.T=pxd... so currJ=dxp. 
    # Also, I is pxp
    # in this case, p=3, d=2
    p=currQ.shape[0]
    d=2 #d is world dimensions
    k=1 #for now its not overdetermined. NOTE: find out if Dylan's paper used overdet.
    currJT = np.zeros((p,d)) 
    I=np.identity(p)
    for i in range(p): #i is a specific robot param
        #the jacobian is first initialized with central differences. 
        '''print("INIT JAC: currQ: ")
        print(currQ)
        print(I[i])
        print("positive perturb", vs_fkin(e, currQ+epsilon*I[i], camera))
        print("negative perturb", vs_fkin(e, currQ-epsilon*I[i], camera))
        print("perturb diff",  (vs_fkin(e, currQ+epsilon*I[i], camera)) - (vs_fkin(e, currQ-epsilon*I[i], camera)))'''
        currJT[i] = (
                    (vs_fkin(e, currQ+epsilon*I[i], camera)) -
                    (vs_fkin(e, currQ-epsilon*I[i], camera))
                ) / (2*epsilon)
        #print("currJT i column", currJT[i])

    currJ=currJT.T
    return currJ

def constjac_3dof(currQ, desiredP, camera: CentralCamera, e : rtb.Robot.ets): #uses a constant Jacobian.
    '''
    CONSTANT JACOBIAN for VISUAL SERVOING

    Steps:
    1. Project the currRP of the EE into the camera and use central differences to initialize the Jacobian. (initialization here and in uncalibrated_vs func are the same!)
    2. 

    '''
    i = 0
    currP=vs_fkin(e, currQ, camera) #projection of that real point through the camera

    errorP=desiredP-currP
    error=np.linalg.norm(errorP)
    if error < TOLERANCE:
        return (i, error, errorP[0], errorP[1], currP, currQ) # our ready pose IS the desired pose

    alpha=0.1

    J=centraldiff_jacobian(currQ, desiredP, camera, e)

    corrQ = np.linalg.pinv(J) @ errorP
    #print("corrQ:",corrQ)
    currQ=currQ+corrQ*alpha
    ###after solvign for the new theta, we move there and check the error.
    currP=vs_fkin(e, currQ, camera)
    errorP=desiredP-currP #error is measured from projection img
    error=np.linalg.norm(errorP)
    i+=1

    while i< MAXITER and error>TOLERANCE:

        currP=vs_fkin(e, currQ, camera)

        errorP = desiredP - currP
        corrQ = -np.linalg.pinv(J) @ errorP 

        error = np.linalg.norm(errorP)
        currQ = currQ + corrQ*alpha #NOTE: we can mult corr step by damping with alpha
        i += 1  # update iteration variable

    Xerror = errorP[0]
    Yerror = errorP[1]
    return i, error, Xerror, Yerror, currP, currQ   

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
   # print("first projected point:", currP)
    #print("this is the desired projected point:", desiredP)
    #print("currQ:", currQ)
    errorP=desiredP-currP
    error=np.linalg.norm(errorP)
    if error < TOLERANCE:
        return (i, error, errorP[0], errorP[1], currP, currQ) # our ready pose IS the desired pose

    epsilon=PI/16
    alpha=0.1

    currJ=centraldiff_jacobian(currQ, desiredP, camera, e)

    #print("currJ:", currJ)
    # NOTE: advised to make f: actual theta -> result image position. double check if mine is OK! :>
    ###now that we have a Jacobian estimate, we can solve for the correction amt in Q.
    corrQ = np.linalg.pinv(currJ) @ errorP
    #print("corrQ:",corrQ)
    currQ=currQ+corrQ
    ###after solvign for the new theta, we move there and check the error.
    currP=vs_fkin(e, currQ, camera)
    errorP=desiredP-currP #error is measured from projection img
    error=np.linalg.norm(errorP)
    i+=1

    #print("currQ:", currQ)

    ### after the initial setup and estimating J, we begin the iterations: 
    while i< MAXITER and error>TOLERANCE:
        #update J using Broyden's method:
        '''
        print("errorP:")
        print (errorP)
        print("errorP@errorP")
        print(errorP@errorP)
        print("currJ:")
        print(currJ)
        print("corrQ")
        print(corrQ)
        print("corrQ.T")
        print(corrQ.T)
        print("(errorP.T - currJ@corrQ):")
        print((errorP.T - currJ@corrQ))
        '''
        #currJ = currJ + alpha* ((errorP.T - currJ@corrQ.T)@corrQ)/(corrQ@corrQ)
        delta_e = errorP - currJ @ corrQ
        currJ += alpha * np.outer(delta_e, corrQ) / (corrQ @ corrQ)


        corrQ = np.linalg.pinv(currJ) @ errorP #calculate the change in theta needed
        currQ=currQ+alpha*corrQ #update the current theta 
        #use the current theta to find our new position and error:
        currP=vs_fkin(e, currQ, camera)
        errorP=desiredP-currP
        Xerror=errorP[0]
        Yerror=errorP[1]
        error=np.linalg.norm(errorP)
        #print("currJ:")
        #print(currJ)
        #print("currQ:")
        #print(currQ)
        i+=1;
    
        #print("currP:", currP)
        #print(" ")
    ### return i, error, pos
    return i, error, Xerror, Yerror, currP, currQ      

def plot_error(e, desiredPP, tolerance, camera, mode):
    '''
    This function plots total error, x error, y error.
    Create a linear space of values of theta between -pi and pi,
    and for each pair of theta values, compute the jacobian and try to converge to desired pos. 
    Then, plot the error for each pair of theta values.


    mode > 10: show slices
    if mode==1 or 11, inverse kinematics
    if mode ==2 or 12, just error from wherever we started
    if mode==3 or 13, then do constant jacobian
    if mode==4 or 14, then simply give position from the set parameters at that point
    '''
    # make a linspace and meshgrid for ranges of q0...qn
    n = 10  #resolution
    dof = e.n

    # Step 1: Create linspaces and meshgrid
    joint_ranges = [np.linspace(-PI/2, PI/2, n) for _ in range(dof)]
    mesh = np.meshgrid(*joint_ranges, indexing='ij')  # ij indexing keeps dimensions aligned

    # Step 2: Stack all grids to create a dof x n x n x ... array
    Q_grid = np.stack(mesh, axis=-1)  # shape: (n, n, ..., n, dof)

    # Step 3: Preallocate error arrays (same shape as grid without dof)
    error_shape = Q_grid.shape[:-1]
    TotalError = np.zeros(error_shape)
    XError = np.zeros(error_shape)
    YError = np.zeros(error_shape)

    # double loop thru q0 and q1, compute inverse kinematics at that starting point and compute the error

    for idx in np.ndindex(error_shape):
                Q =Q_grid[idx]
                if mode>10: #get the slices
                    slices=1
                else:
                    slices=0
                if (mode-10)==1 or mode==1: #use uncalibrated visual servoing inverse kinematics, init J central diff, update w Broyden's
                    iterations, TotalError[idx], XError[idx], YError[idx], position, theta = uncalibrated_vs(Q, desiredPP, camera, e)
                if (mode-10)==2 or mode==2: #directly measure error from q0 and q1
                    realPos = fkin3D(e, Q)
                    TotalError[idx] = np.linalg.norm(desiredRP-realPos)
                    XError[idx] = (desiredRP-realPos)[0]
                    YError[idx] = (desiredRP-realPos)[1]
                if (mode-10)==3 or mode==3: #use a constant jacobian
                    iterations, TotalError[idx], XError[idx], YError[idx], position, theta = constjac_3dof(Q, desiredPP, camera, e)
                if (mode-10)==4 or mode==4:
                    TotalError[idx]=fkin3D(e, Q)[1]
                    print(fkin3D(e, Q))
                    print(TotalError[idx])

    
    print("datapoints collected. plotting...")
    Q_flat = Q_grid.reshape(-1, dof)  # shape: (N, dof)
    x=Q_flat[:,0]
    y=Q_flat[:,1]
    z=Q_flat[:,2]
    TotErrors=TotalError.flatten()

    if slices==1:
        print("SLICES")
        # animation to iter through q0 slices!
        unique_z_vals = np.unique(np.round(z, 3)) #round the z TotErrors in the flattened space to 3 decimals, and then get each unique val. basically same thing as z, but make it more compact
        slice_thickness = 0.01
        frames = []


        for z_val in unique_z_vals:
            indices = np.where(np.isclose(z, z_val, atol=slice_thickness))[0] 
            if len(indices) == 0:
                continue
            frames.append(go.Frame(
                data=[
                    go.Scatter(
                        x=x[indices],
                        y=y[indices],
                        mode='markers',
                        marker=dict(
                            size=15,
                            color=TotErrors[indices],
                            colorscale='plasma',
                            cmin=min(TotErrors),
                            cmax=max(TotErrors),
                            colorbar=dict(title='Error'),
                            opacity=0.8
                        )
                    )
                ],
                name=str(z_val)
            ))

        # Set initial frame
        initial_indices = np.where(np.isclose(z, unique_z_vals[0], atol=slice_thickness))[0]

        # Main figure layout
        fig = go.Figure( #this is the first frame
            data=[
                go.Scatter( 
                    x=x[initial_indices],
                    y=y[initial_indices],
                    mode='markers',
                    marker=dict(
                        size=15, 
                        color=TotErrors[initial_indices],
                        colorscale='plasma',
                        cmin=min(TotErrors),
                        cmax=max(TotErrors),
                        colorbar=dict(title='Error'),
                        opacity=0.8
                    )
                )
            ],
            layout=go.Layout(
                title='Z-Slice Viewer (q2 slice)',
                xaxis_title='q0',
                yaxis_title='q1',
                width=700,
                height=600,
                updatemenus=[dict(
                    type='buttons',
                    showactive=False,
                    buttons=[dict(label='Play', method='animate', args=[None])]
                )],
                sliders=[dict(
                    steps=[dict(method='animate',
                                args=[[str(zv)], dict(mode='immediate', frame=dict(duration=0), transition=dict(duration=0))],
                                label=f"{zv:.2f}") for zv in unique_z_vals],
                    active=0,
                    transition=dict(duration=0),
                    x=0.1,
                    xanchor="left",
                    y=0,
                    yanchor="top"
                )]
            ),
            frames=frames
        )
    
    else: #not slices
        print("3D PLOT")
        scatter = go.Scatter3d(x=x,y=y,z=z,mode='markers', marker=dict(size=15,color=TotErrors, colorscale='plasma', colorbar=dict(title='Total Error Non-Normalized'), opacity=0.2))
        layout = go.Layout(
            scene=dict(
                xaxis_title='q0',
                yaxis_title='q1',
                zaxis_title='q2'
            ),
            title='Joint Space Error in 3D',
            margin=dict(l=0,r=0,b=0,t=50)
        )
        fig=go.Figure(data=[scatter], layout=layout)

    fig.show()

def main():
    camera : CentralCamera = init_camera(f, rho, u0, v0, 1000)

    #NOTE: desiredRP is a global variable
    desiredPP = proj_point(desiredRP, camera)

    #just use ets. but if we want to use a robot, we can simply transform the robot into an ets
    ets0 = rtb.ET.Rz() * rtb.ET.tx(l0) * rtb.ET.Rz() * rtb.ET.tx(l1) #2DOF, x-y plane, very simple. 
    ets1 = rtb.ET.Rz() * rtb.ET.tx(l0) * rtb.ET.Rx() * rtb.ET.tz(l1) #z-axis revolute and another y-axis revolute jnt (DEPTH TESTER)
    ets2= rtb.ET.Rz() * rtb.ET.tx(l0) * rtb.ET.Rx() * rtb.ET.tz(l1) * rtb.ET.Rx() * rtb.ET.tz(l2)
    ets=ets2
    '''
    ee_world = fkin3D(ets, initialQ) # Position in real world coords: (0.88, 1.19, 0)
    print("TEST WORLD POINT", ee_world)

    ee_cam = h2e(np.linalg.inv(camera.T.A) @ e2h(ee_world)) #Position in the real world with z as distance from object to camera: (0.88, 1.19, 5.0)
    print("TEST WORLD POINT but Z is dist from camera to object:", ee_cam)

    desired_cam = h2e(np.linalg.inv(camera.T.A) @ e2h(desiredRP)) #this should be 1,1,5
    print("DESIRED POINT but Z is dist from camera to object:", desired_cam)

    #So far so good. Check if the projection is crazy rn
    projected_desired_point = proj_point(desiredRP, camera)
    print("DESIRED PROJ P", projected_desired_point)
    '''


    
    iterations, error, Xerror, Yerror, position, theta = uncalibrated_vs(initialQ, desiredPP, camera, ets)
    print("iterations:", iterations,"error:", error)
    print("DESIRED projected pos:", desiredPP, "RESULT projected position:", position)
    print("DESIRED real pos:", desiredRP, "RESULT real position:",fkin3D(ets, theta))
    '''
    robot=rtb.Robot(ets)
    q=theta
    print(q)
    robot.plot(q,block=True)'''

    print("Creating plot of q0 vs q1 vs position error, for visual servoing.")
    plot_error(ets, desiredPP, TOLERANCE, camera, 13)

main()

