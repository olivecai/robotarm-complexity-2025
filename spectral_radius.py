'''
June 4 2025

Exploring spectral radius and singular Jacobians.

############# OVERVIEW #############
The spectral radius (referred to as p here) can tell us how stable a system is:
p < 1 : matrix converges
else, matrix diverges.

a "convergent matrix" converges to the zero matrix under matrix exponentiation.
### read about convergent matrixes here: https://en.wikipedia.org/wiki/Convergent_matrix 

Qn+1 = g(Qn), where Qk is joint parameters at iteration k, and g(...) is the update function, as defined below:
gn(Qn) = Qn - BF(Qn)
    B  is the J_inverse calculated at the starting position through central differences, and
    F  is the residual function for the fkin error: F(Qn) = fkin(Qn)-desired_position_cartesian_coords = current-desired

in Newton's method, we have successful convergence when B @ (del F(Qn) / del Qn) == I:
del gn / del Qn = I - B @ (del F(Qn) / del Qn)

For ease in notation, we will refer to del gn / del Qn as matrix 'A'.
We will also refer to del F(Qn) / del Qn as matrix 'dF'
Matrix A is the Jacobian of the update function, gn. 
    It tells us how small changes in the current guess Qn affect the next guess.
    
############# EXECUTION #############
Simple experiment:
Using symbolic language libraries, get the derivatives and Jacobians for a single point that we KNOW converges with a constant Jacobian.
Get the spectral radius.
Repeat for a starting point that does NOT converge.
Repeat for a starting point which may pass through a singularity.
Repeat for a starting point at a singularity.

Expanding experiements:
Look into the joint spectral radius over a trajectory.
Focus on what the spectral radius can tell us that the Jacobian cannot 

B is only an approximation,
while 
'''

import sympy as sp
import successive_mesh_refinement as smr
import common_robot_calculations as crc
import numpy as np
import roboticstoolbox as rtb

        
def spectral_radius(u_, v_):
    '''
    
    '''
     # u and v <==> theta1 and theta2
    u = sp.Symbol('u', real=True)
    v = sp.Symbol('v', real=True)
    # l1 and l2 
    l1 = sp.Symbol('l1', positive=True)
    l2 = sp.Symbol('l2', positive=True)
    l1=1;l2=1; #for ease let's just have lengths be 1

    #desired position
    x0 = sp.Symbol('x0', real=True)
    y0 = sp.Symbol('y0', real=True)
    x0=1
    y0=1
    desired = sp.Matrix([x0,y0])

    #declare the forward kin functions
    f1 = l1 * sp.cos(u) + l2 * sp.cos(u+v)
    f2 = l1 * sp.sin(u) + l2 * sp.sin(u+v)
    
    ets2dof = rtb.ET.Rz() * rtb.ET.tx(l1) * rtb.ET.Rz() * rtb.ET.tx(l2) 
    joint_limits2dof = [(-np.pi/2, np.pi/2), (-np.pi/2, np.pi/2)]  # example for 2 DOF
    joint_limits2dof_full = [(-2*np.pi, 2*np.pi), (-2*np.pi, 2*np.pi)]
    joint_limits = joint_limits2dof
    joint_limits_full = joint_limits2dof_full
    ets=ets2dof
    robot = rtb.Robot(ets)
    camera=None 
    mesh = smr.DelaunayMesh(1e-1, robot, camera, sparse_step=10, jointlimits=joint_limits_full)
    smr.create_sparsespace(mesh)
    smr.create_delaunaymesh_2DOF(mesh, 1) #NOTE: this line is NOT necessary for computations, it is simply to show the viewer our plot.
    smr.calculate_simplices(mesh) #this actually calculates the simplices we need.
    smr.create_mesh_jacobians(mesh, ets, 1)

    f = sp.Matrix([f1, f2]) #forward kinematics matrix
    
    reps = [(u, u_), (v, v_)]

    #We have a few options to get the Jacobian here...
    mode=1
    while mode<4:
        if mode==1:
            J = f.jacobian([u,v]) #ANALYTIC jacobian of the forward kinematics function
            J=J.subs(reps)
            B = J.inv() #ANALYTIC
        if mode==2: #simplex designated jacobian
            
            J=np.delete(mesh.mesh_jacobians[smr.find_simplex(np.array([u_,v_]),mesh)], 2, axis=0)
            B=np.linalg.inv(J)
        if mode==3:
            J=np.delete(crc.centraldiff_jacobian(np.array([u_,v_]),ets), 2, axis=0)
            B=np.linalg.inv(J)
        print(J)
        mode+=1;

    F = f - desired #current-desired #AMBIGUOUS IN VISUAL SERVOING
    dF = F.jacobian([u,v])

    I = sp.eye(2) #hardcoded to 2 for now, while we run experiments on the 2DOF planar arm

    A = I - B*dF

    print(A.subs(reps))

    sr=calculate_spectral_radius(A.subs(reps))

    return sr



def calculate_spectral_radius(A):
    # Get the eigenvalues
    eigenvals = A.eigenvals()  # returns dict: {eigenvalue: multiplicity}
    # Compute the spectral radius (max absolute eigenvalue)
    return sp.Max(*[sp.Abs(lam) for lam in eigenvals.keys()])

def main():

    u_ = 0 #sp.pi/2
    v_ = 1.570796 #sp.pi
    

    #TODO: compute the spectral radius over the joint space

#So, there are two things we need to consider: estimating both B and dF
    




main()



