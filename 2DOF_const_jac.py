'''
May 2025
2D, 2DOF robot arm simulation: find where the robot converges when Jacobian is constant.
'''
import numpy as np
from spatialmath import SE3
from spatialmath.base import sym
import roboticstoolbox as rtb
import matplotlib.pyplot as plt
PI=np.pi
SIN=np.sin
COS=np.cos
DESIRED=np.array([1,1])
l0=1 #hardcode lengths of arms
l1=1

def get_pos(e, theta):
    ''' NOTE: idxing is for SE3 translation/rotation matrix, not 2D!!!'''
    T= e.eval(theta)
    x=T[0][3]; 
    y=T[1][3];
    return np.array([x,y]);

def inverse_kinematics_2d(e, theta, desired_pos, tolerance, maxiter, J): #uses a constant Jacobian. So, do this step for every q0 q1 in our meshgrid?
    def norm(v):
        return np.linalg.norm(v)

    old_pos=get_pos(e, theta)
    error = norm(old_pos - desired_pos)

    iter_var = 0

    while iter_var < maxiter and error > tolerance:
        iter_var += 1  # update iteration variable
        old_theta = np.copy(theta) #update theta

        old_pos=get_pos(e, old_theta)

        residual = old_pos - desired_pos
        correction_step = -np.linalg.pinv(J) @ residual 

        error = norm(residual)
        theta = old_theta + correction_step #NOTE: we can mult corr step by damping

    xerror = residual[0]
    yerror = residual[1]
    return error, xerror, yerror

def errorchart(e, desired, tolerance, maxiter, J):
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
            J = np.array([
                        [-l0 * SIN(q0) - l1 * SIN(q0 + q1), -l1 * SIN(q0 + q1)],
                        [ l0 * COS(q0) + l1 * COS(q0 + q1),  l1 * COS(q0 + q1)]
                        ]);
            TotalError[i,j], XError[i,j], YError[i,j] = inverse_kinematics_2d(e, np.array([q0,q1]), desired, tolerance, maxiter, J) #calculate the error of this particular q0 and q1 and find the inverse kinematics

    #Plot the Error Surface 
    fig = plt.figure(figsize=(18, 6))

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

    plt.tight_layout()
    plt.show()


def main():
    q0=1 #hardcode test angles for now
    q1=1
    theta=np.array([q0,q1]);
    e2 = rtb.ET.Rz() * rtb.ET.tx(l0) * rtb.ET.Rz() * rtb.ET.tx(l1) #each arm is 1m long.
    T= e2.eval(theta)
    x=T[0][3]; #get the individual positions x and y 
    y=T[1][3];
    #note that z would be T[2][3], but it will always be 0 in this case

    print(T)
    print(x,y)

    J = np.array([
        [-l0 * SIN(q0) - l1 * SIN(q0 + q1), -l1 * SIN(q0 + q1)],
        [ l0 * COS(q0) + l1 * COS(q0 + q1),  l1 * COS(q0 + q1)]
        ]);

    print(J)

    #Inverse kinematics: try to make this code as modular as possible! Make jacobian constant. If the difference between the const_jac pos and the actual pos are increasing after 5 iterations, can we assume it will fail?
    maxiter=30; tolerance=1e-6;
    
    # NOTE: the next two lines were just setting up the inv kin 2D for one theta pair. Now let's go on to iterate over our linspace so we can make a graph 
    #error = inverse_kinematics_2d(e2, theta, DESIRED,tolerance, maxiter, J) #inv kin using const jac
    #print("this is the error:"); print(error)
    errorchart(e2, DESIRED, tolerance, maxiter, J)

def testing():
    n=100;
    quad_range=np.linspace(0,50,n)
    Q0, Q1 = np.meshgrid(quad_range, quad_range) 
    Z = np.zeros_like(Q0)
    for i in range (n):
        for j in range(n):
            Z[i,j] = i*i

    #Plot the Error Surface 
    fig = plt.figure(figsize=(6,6))

    # First plot: total error
    ax1 = fig.add_subplot(1, 1, 1, projection='3d')
    ax1.plot_surface(Q0, Q1, Z, cmap='plasma')
    ax1.set_xlabel("x")
    ax1.set_ylabel("y")
    ax1.set_zlabel("Z")
    ax1.set_title("testy test")

    plt.show()
    



#testing();
main();
print("End of program.")

#next steps:
# see how far we are
#is there some way to graph ...... error, theta1, theta2? we need a 3d plot and some datapoints... hmm

#first we need to collect a lot of datapoints. should i do a scatterplot or a smooth curve? can i even make a smooth curve right now?


