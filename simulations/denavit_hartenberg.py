'''
July 15

This program obtains analytic expressions from the Denavit Hartenberg parameters.
'''

import sympy as sp
import numpy as np
import roboticstoolbox as rtb
from scipy.optimize import minimize

class DHSympyParams:
    '''
    these must be initialized first so that we can use the parameters in the dh system
    '''
    def __init__(self):
        self.joint_vars = sp.symbols('t(:10)') #t1,t2,t3,...t9
        t0,t1,t2,t3,t4,t5,t6,t7,t8,t9 = self. joint_vars
        self.cart_space_vars = sp.symbols('x,y,z') #x,y,z
        x,y,z = self.cart_space_vars
        self.task_space_vars = sp.symbols('u,v') #u,v
        u, v = self.task_space_vars
        
    def get_params(self):
        return (self.joint_vars, self.cart_space_vars, self.task_space_vars)
    
# for a desired position, what is the analytic Newton's equation?

# u and v <==> theta1 and theta2
u,v,w = sp.symbols('u:w', real=True)

# l1 and l2 
l1,l2,l3 = sp.symbols('l(1:4)', positive=True)

#CAMERA 
fx = sp.Symbol('fx', real=True)
fy = sp.Symbol('fy', real=True)
cx = sp.Symbol('cx', real=True)
cy = sp.Symbol('cy', real=True)

class Camera:
    '''
    creates a camera with actual parameters
    '''
    def __init__(self, rot_xaxis, rot_yaxis, rot_zaxis, translation, fx, fy, cx, cy):
        '''
        rot axis is a radian rotation around a specified axis.
        '''
        self.K=sp.Matrix([[fx, 0, cx],[0, fy, cy], [0,0,1]]) #intrinsic matrix

        rx = sp.Matrix([[1,0,0],[0,sp.cos(rot_xaxis), -sp.sin(rot_xaxis)],[0,sp.sin(rot_xaxis), sp.cos(rot_xaxis)]])
        ry= sp.Matrix([[sp.cos(rot_yaxis), 0, sp.sin(rot_yaxis)],[0,1,0],[-sp.sin(rot_yaxis), 0, sp.cos(rot_yaxis)]])
        rz = sp.Matrix([[sp.cos(rot_zaxis), -sp.sin(rot_zaxis), 0], [sp.sin(rot_zaxis), sp.cos(rot_zaxis),0],[0,0,1]])
        
        R = rx*ry*rz
        t=sp.Matrix([translation[0],translation[1],translation[2]])
        
        E = R.col_insert(3,t)

        self.E = E

        self.P = self.K*self.E

    def projectpoint(self, worldpoint):
        '''
        Projects a sympy Matrix object of shape (4, 1)
        '''
        worldpoint=sp.Matrix(worldpoint)
        if worldpoint.shape[0] != self.P.shape[1]:
            worldpoint = worldpoint.row_insert(worldpoint.shape[0], sp.Matrix([[1]]))
        try:
            x = self.P * worldpoint
        except:
            print("EXCEPTION:", worldpoint)
        #print("projection point before flatten:")
        #print(x)
        x[0]=x[0]/x[2]
        x[1]=x[1]/x[2]
        x[2]=1#'''
        return sp.Matrix([[x[0]],[x[1]]])

class DenavitHartenbergAnalytic():
    '''
    initialize a denavit hartenberg system with analytic symbols, using sympy.
    '''
    def __init__(self, dh_params: list, symbolclass: DHSympyParams):
        self.cartvars = symbolclass.cart_space_vars
        self.taskvars = symbolclass.task_space_vars
        self.jntvars = symbolclass.joint_vars
        #dh_params is a double nested list, where each row is one joint.

        self.lipschitz = None
        
        transforms=[]
        for param in dh_params: # for each joint 
            transforms.append(self.transformation_matrix_DH(*param)) #transformation_matrix_DH returns the DH matrix for that single joint...
        ee_matrix = sp.eye(4)
        for i in range(len(transforms)): #chain multiply the DH matrices to get the final position
            ee_matrix *= transforms[i]

        ee_translation = ee_matrix[:,3][:3] #translation final position 
        self.ee_matrix = ee_matrix #entire matrix including rotations etc
        self.ee_translation = sp.Matrix(ee_translation)
        self.dof = len(dh_params) #degree of freedom correlates to the number of dh parameter rows

        self.dh_params = dh_params
        self.rtb_robot = self.rtb_model()

        self.jointlimits = [(0, np.pi/2)] * self.dof

        #print("ee_translation:", self.ee_translation)
        self.F = sp.Matrix(self.ee_translation[:3]) - sp.Matrix(self.cartvars)
        #print("F:", self.F)
        
        self.J_analytic = sp.Matrix(self.ee_translation[:3]).jacobian(self.jntvars[:self.dof])
        self.J = (sp.utilities.lambdify(self.jntvars[:self.dof], self.J_analytic, 'numpy'))
        

        variables = self.jntvars[: self.dof] + self.cartvars
        self.fkin_eval = (sp.utilities.lambdify(self.jntvars[:self.dof], self.ee_translation, 'numpy'))
        self.errfn_eval= (sp.utilities.lambdify(variables, self.F, 'numpy'))

    def lipschitz_objective(self, q):
        # Objective: negative spectral norm (so that minimize → maximum)
        fn = (sp.utilities.lambdify(self.jntvars[:self.dof], self.J, 'numpy'))
        J_val = fn(*q)
        norm_val = np.linalg.norm(J_val, ord=2).copy()
        #print("J:\n", J_val)
        #print("q:", q, "→ norm:", norm_val)
        return -norm_val
    
    def calc_lipschitz(self):
        # Initial guess and bounds
        q0 = np.array([0.1]*self.dof) #whatever is the maximum 
        bounds = [(0, np.pi)]*self.dof
        print("BOUNDS")
        print(bounds)

        res = minimize(self.lipschitz_objective, q0, bounds=bounds)
        L_estimate = -res.fun

        self.lipschitz = L_estimate
        return L_estimate

    def ret_kantovorich(self, initQ, desP, alpha, lipschitz=None,):
        
        reps_des = []
        reps_dof = []

   
        
        for i in range(len(desP)):
            reps_des.append((self.cartvars[i], desP[i]))
        for i in range(self.dof):
            reps_dof.append((self.jntvars[i], initQ[i]))

        self.alpha = alpha
        #print(reps_des)
        F = np.array(self.F.subs(reps_des).subs(reps_dof), dtype=float).flatten()

        print("F", F)

        J = self.central_differences(initQ, desP)
        print(J)

        JI=np.linalg.pinv(J)
        print("JI", JI)
        
        JI_F = np.array(JI@F, dtype=float).flatten()
        print(np.array(JI_F))

        b = np.linalg.norm(JI_F, ord=2) #norm of newton step, ƞ
        b= np.linalg.norm(np.array(F)) #NOTE ht kung uses this as condition

        B = np.linalg.norm(JI, ord=2) #norm of jacobian inverse to determine if jacobian itself is nonsingular
        B = np.log10(B) #it seems that reducing this helps 

        print("b:", b, "B:", B)

        spectral_norm = np.linalg.norm(J, ord=2)
        inv_sn = np.linalg.norm(JI, ord=2)

        cond = spectral_norm * inv_sn

        print("CONDITION NUMBER:", cond) # if the condition number or B is super high, we are at a singularity and we need to move.
        # note that the condition number is always >=1, since it is defined as the greatest singular value of A / smallest singular value of A

        print("SPECTRAL NORM:", spectral_norm)

        if lipschitz is None:
            h = spectral_norm * b * B #NOTE: h should be = global_lipschitz * b * B, but since the fkin fn is relatively smooth, this should be okay
        else:
            h = lipschitz * b * B

        print("h:", h)

        if h == 1/2:
            p = (1-np.sqrt(1-2*h))*b/h
        elif h < 1/2:
            p = (1+np.sqrt(1-2*h))*b/h
        else: # h > 1/2
            p = None

        return h, p, b, B


    def transformation_matrix_DH(self, theta_i, alpha_i, r_i, d_i):
        '''
        Returns the general denavit hartenberg transformation matrix for one link, i.
        theta_i is the angle about the z(i-1) axis between x(i-1) and x(i).
        alpha_i is the angle about the x(i) axis between z(i-1) and z(i).
        r_i is the distance between the origin of frame i-1 and i along the x(i) direction.
        d_i is the distance from x(i-1) to x(i) along the z(i-1) direction.
        '''
        alpha, r, theta, d = sp.symbols('alpha, r, theta, d', real=True)
        general_DH = sp.Matrix([[sp.cos(theta), -sp.sin(theta)*sp.cos(alpha), sp.sin(theta)*sp.sin(alpha), r*sp.cos(theta)],
                                [sp.sin(theta), sp.cos(theta)*sp.cos(alpha), -sp.cos(theta)*sp.sin(alpha), r*sp.sin(theta)],
                                [0, sp.sin(alpha), sp.cos(alpha), d],
                                [0,0,0,1]])
        #print(general_DH)
        DH = general_DH.subs([(alpha, alpha_i), (r, r_i), (theta, theta_i), (d, d_i)])
        #print(DH)
        return DH

    def central_differences(self, Q, desP, epsilon=None):
        '''
        Returns the central differences Jacobian and the value of epsilon to perturb by
        '''
        Q= np.array((Q))

        if epsilon == None:
            epsilon = 1e-4
        
        p= Q.shape[0]
        d= self.F.shape[0]
        #print("pxd:", p,"x",d)

        k=1
        Jt = np.zeros((p,d))
        I = np.identity(p)
    
        for i in range(p):
     
            forward = self.errfn_eval(*(Q + epsilon * I[i]) , *desP)
            #print(forward)
            backward = self.errfn_eval(*(Q - epsilon * I[i]) , *desP)
            #print(backward)
            diff = (forward-backward).T
            #print(diff)
            Jt[i] = diff / (2*epsilon)

        return Jt.T
    
    def const_jac_inv_kin(self, desP, initQ):
        '''
        Q = initQ
        
        '''
        # self.F = fkin_x - x, fkin_y - y, fkin_z - z
        currQ = initQ
        tolerance = 1e-3
        maxiter= 400

        traj = [currQ]
        
        J = self.central_differences(currQ, desP)

        ret= -1
        F = self.F
        reps_des = []
        for i in range(len(desP)):
            reps_des.append((self.cartvars[i], desP[i]))
        F = F.subs(reps_des)

        for i in range(maxiter):

            reps_dof = []
            
            for j in range(self.dof):
                reps_dof.append((self.jntvars[j], currQ[j]))

            currError = np.array(F.subs(reps_dof).evalf()).astype(np.float64) #july 28 im p sure we could jsut do errfn_eval i dont rememebr why i did this to myself
            #print("currError:", currError.flatten())

            if np.linalg.norm(currError) <= tolerance:
                ret = i
                break

            #print(i, end=': ')
            
            newtonStep = (np.linalg.pinv(J) @ currError).flatten()
            #print("newtonStep", newtonStep)
            currQ = currQ - self.alpha * newtonStep
            #print("currQ:", currQ)
            traj.append(currQ)

        traj=np.array(traj)
        if 1:
            self.rtb_robot.plot(traj, block=False)
            
        return ret

    def rtb_model(self):
        '''
        Just to verify everything is okay, I have the rtb DH robot initialized here.
        '''
        links = []

        for i in range(len(self.dh_params)):
            for j in range(4):
                try:
                    self.dh_params[i][j] = float(self.dh_params[i][j])
                except:
                    pass
            
            links.append(rtb.RevoluteDH(alpha = self.dh_params[i][1], a=self.dh_params[i][2], d=self.dh_params[i][3]))

        robot = rtb.DHRobot(links, name=f"robot_{self.dof}dof")
        print(robot.dhunique())
        print(robot)

        return robot

    def plot(self, Q=None):
        if Q is None:
            print("plotting robot joints zero vector")
            Q = [0.0] * self.dof
        self.rtb_robot.plot(Q, block=True)
        

if __name__ == '__main__':
    P = DHSympyParams()
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
                    [ t2,  0  ,  0.30, 0 ]
                    ]

    #dof2 = DenavitHartenbergAnalytic(dof2_params, P)
    dof3 = DenavitHartenbergAnalytic(dylan_dof3_params, P)

   
class DenavitHartenberg_Cameras_Analytic():
    '''
    initialize a denavit hartenberg system plus camera(s) with analytic symbols, using sympy.

    Compare the lipschitz constant of with vs without cameras to see whether cameras majorly affect the complexity (hopefully they do not.)

    '''
    # it is important to get the error function F to reduce to 0
    def __init__(self, cameras: list, dh_robot: DenavitHartenbergAnalytic):
        '''
        cameras is a list of Camera objects
        dh_robot is a DenavitHartenbergAnalytic object
        '''
        if type(cameras) is not list:
            cameras = list(cameras)

        self.cameras =cameras
        self.dh_robot = dh_robot
        self.jntvars, self.cartvars, self.taskvars = dh_robot.jntvars, dh_robot.cartvars, dh_robot.taskvars

        self.F = []# is a factor of 2, since each camera gives two projected points.
        for camera in cameras:
            projected_point = camera.projectpoint(self.dh_robot.F)
            self.F.append(projected_point)  
        self.F = sp.Matrix(self.F)

        self.J = self.F.jacobian(self.dh_robot.jntvars[:self.dh_robot.dof])

        variables = dh_robot.jntvars[: dh_robot.dof] + dh_robot.cartvars
        self.errfn_eval= (sp.utilities.lambdify(variables, self.F, 'numpy')) #this uses the TRUE desired point... but that's not accessible in real VS, so we should move away from this function
        self.jacobian_eval = (sp.utilities.lambdify(variables, self.J, 'numpy')) #ah, same goes for this function...
        #now self.F should be the equation of the projection, errfn_eval will evaluate F given the params

        self.lipschitz=None

        self.error_fn_eval = None #

    def set_error_fn(self, error_equation_eval):
        '''
        as of aug 5 NOT COMPLETED, think im going to try different constraints directly on kinova instead
        '''
        self.err_fn_eval = error_equation_eval #already projected through the cameras?
        '''
        For instance, for a parallel line to line constraint with two point to point constraints, we might have:

        point A - point a == 0
        point B - point b == 0
        Line A-B cross product with line a-b == 0

        And if point A is the end effector and point B is the second last joint, we have 

        projection of end effector - projection of world point a = 0
        projection of second last jnt - projection of world point b = 0
        projection of (B-A) X projection of (b-a) = 0 

        But we should specify A and B in terms of variables of the vector Q.

        So, we pass the error equation vector as a sympy object,
        Variables will be: the joint vector Q, and then after the desired constraint points.

        The question is should we pass real or projected points? I think projected points.
        '''

    def central_differences_pp(self, Q, tracked_points, epsilon=None):
        '''
        Pass the PROJECTED POINTS directly, NOT the real point. 

        Returns the central differences Jacobian and the value of epsilon to perturb by

        the matrix J should be (number of cameras * 2) x (dof)
        '''

        Q= np.array((Q))

        if epsilon == None:
            epsilon = 10e-1
        
        p= Q.shape[0]
        d= self.F.shape[0]
        #print("pxd:", p,"x",d)

        k=1
        Jt = np.zeros((p,d))
        I = np.identity(p)
    
        for i in range(p):
            
            forward = self.projected_world_point(self.dh_robot.fkin_eval(*(Q + epsilon * I[i])))
            
            backward = self.projected_world_point(self.dh_robot.fkin_eval(*(Q - epsilon * I[i])))
            

            diff = np.array((forward-backward)).T
            #print(diff)
            Jt[i] = diff / (2*epsilon)

        return -Jt.T
        
    def projected_errfn_eval(self, initQ, desPP):
        #for each camera project the end effector point and subtract the desired point
        errfn = []
        eeRP = self.dh_robot.fkin_eval(*initQ)
        eePP = self.projected_world_point(eeRP)

        errfn = np.subtract(desPP, eePP)

        return errfn
    

    def const_jac_inv_kin_pp(self, desP, initQ, J=None):
        '''
        desP is a PROJECTED POINT through the camera already!!!!!

        Q = initQ
        
        '''
        print("Newton Method")

        # self.F = fkin_x - x, fkin_y - y, fkin_z - z
        currQ = initQ
        tolerance = 1e-3
        maxiter= 30

        traj = [currQ]
        
        if J is None:
            J = self.central_differences_pp(currQ, desP)
        print("Jacobian:\n", J)

        errors=[]

        ret= -1

        for i in range(maxiter):
            print("i:", i)
            currError = self.projected_errfn_eval(currQ, desP) #first calculate the error
            #print("currError:", currError.flatten())
            print("current error:\n", currError)
            errors.append(currError)

            if np.linalg.norm(currError) <= tolerance: #if error is near 0 then we can end the iterations
                ret = i
                break
            
            Jinv =np.linalg.pinv(J) 
            print("J inverse:", Jinv)
            print("Norm of J Inv:", np.linalg.norm(Jinv))

            newtonStep = (Jinv @ currError).flatten()
            print("newtonStep\n", newtonStep)
            currQ = currQ - self.dh_robot.alpha * newtonStep

            print("currQ:\n", currQ)
            traj.append(currQ)

        traj=np.array(traj)
        if 1:
            self.dh_robot.rtb_robot.plot(traj, block=False)
            
        

        return ret, currQ
    
    
    def central_differences_pp(self, Q, desPP, epsilon=None):
        '''
        Pass the PROJECTED POINTS directly, NOT the real point. 

        Returns the central differences Jacobian and the value of epsilon to perturb by

        the matrix J should be (number of cameras * 2) x (dof)
        '''

        Q= np.array((Q))

        if epsilon == None:
            epsilon = 10e-1
        
        p= Q.shape[0]
        d= self.F.shape[0]
        #print("pxd:", p,"x",d)

        k=1
        Jt = np.zeros((p,d))
        I = np.identity(p)
    
        for i in range(p):
            
            forward = self.projected_world_point(self.dh_robot.fkin_eval(*(Q + epsilon * I[i])))
            
            backward = self.projected_world_point(self.dh_robot.fkin_eval(*(Q - epsilon * I[i])))
            

            diff = np.array((forward-backward)).T
            #print(diff)
            Jt[i] = diff / (2*epsilon)

        return -Jt.T

    def projected_world_point(self, real_world_point):
        '''
        project a real world point and get the image projection in all cameras
        '''
        #print("@@@@@@@@@@@@@@@@@@@@@@@@\n", real_world_point)
        projected_points =[]
        for camera in self.cameras:
            proj_pnt = camera.projectpoint(real_world_point)
            projected_points.append(np.array(proj_pnt, dtype=float).flatten())

        return (np.array(projected_points).flatten())
    

    def lipschitz_objective(self, q):
        # Objective: negative spectral norm (so that minimize → maximum)
        reps=[]
        for i in range(len(self.dh_robot.cartvars)):
            reps.append((self.dh_robot.cartvars[i], 10))

        fn = (sp.utilities.lambdify(self.dh_robot.jntvars[:self.dh_robot.dof], self.J.subs(reps), 'numpy'))
        J_val = fn(*q)

        norm_val = np.linalg.norm(J_val, ord=2).copy()
        
       

        #print("J:\n", J_val)
        #print("q:", q, "→ norm:", norm_val)
        return -norm_val
    
    def calc_lipschitz(self, Lipschitz=None):
        # Initial guess and bounds

        if Lipschitz:
            self.lipschitz = Lipschitz #if we already calculated this and dont want to again...
            return Lipschitz
        
        q0 = np.array([0.1]*self.dh_robot.dof) #whatever is the maximum 
        bounds = [(0, np.pi)]*self.dh_robot.dof
        #print("BOUNDS")
        #print(bounds)

        res = minimize(self.lipschitz_objective, q0, bounds=bounds)
        L_estimate = -res.fun

        self.lipschitz = L_estimate       
        print("Calculated Lipschitz Constant:", L_estimate)

        return L_estimate
    
    def central_differences(self, Q, desP, epsilon=None):
        '''
        DesP is the REAL WORLD POINT.
        Returns the central differences Jacobian and the value of epsilon to perturb by

        the matrix J should be (number of cameras * 2) x (dof)
        '''
        Q= np.array((Q))

        if epsilon == None:
            epsilon = 1e-1
        
        p= Q.shape[0]
        d= self.F.shape[0]
        #print("pxd:", p,"x",d)

        k=1
        Jt = np.zeros((p,d))
        I = np.identity(p)
    
        for i in range(p):
     
            forward = self.errfn_eval(*(Q + epsilon * I[i]) , *desP)
            print("f", forward)
            backward = self.errfn_eval(*(Q - epsilon * I[i]) , *desP)
            print("b", backward)
            diff = (forward-backward).T
            #print(diff)
            Jt[i] = diff / (2*epsilon)

        return Jt.T
    
    


        


    

# initialize a system with cameras and see the lipschitz constant

# initialize a system without cameras and see lipschitz constant

# SET UP INVERSE KINEMATICS:
# desired position, initial position

# for the given inverse kinematics scenario:
# run newtons method
# run ht_kung function 
# run MY method


