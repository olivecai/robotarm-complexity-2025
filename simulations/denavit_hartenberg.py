'''
July 15

This program obtains analytic expressions from the Denavit Hartenberg parameters.
'''

import sympy as sp
import numpy as np
import roboticstoolbox as rtb

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

class DenavitHartenbergAnalytic():
    '''
    initialize a denavit hartenberg system with analytic symbols, using sympy.
    '''
    def __init__(self, dh_params: list, symbolclass: DHSympyParams):
        self.cartvars = symbolclass.cart_space_vars
        self.taskvars = symbolclass.task_space_vars
        self.jntvars = symbolclass.joint_vars
        #dh_params is a double nested list, where each row is one joint.
        
        
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

        print("ee_translation:", self.ee_translation)
        self.F = sp.Matrix(self.ee_translation[:3]) - sp.Matrix(self.cartvars)
        print("F:", self.F)
        
        self.J = self.F.jacobian(self.jntvars[:self.dof])

        variables = self.jntvars[: self.dof] + self.cartvars
        self.fkin_eval = (sp.utilities.lambdify(self.jntvars[:self.dof], self.ee_translation, 'numpy'))

    def ret_kantovorich(self, lipschitz, initQ, desP, alpha):
        
        reps_des = []
        reps_dof = []
        
        for i in range(len(desP)):
            reps_des.append((self.cartvars[i], desP[i]))
        for i in range(self.dof):
            reps_dof.append((self.jntvars[i], initQ[i]))

        self.alpha = 1
        print(reps_des)
        F = self.F.subs(reps_des).subs(reps_dof)

        print("F", F)

        J = self.central_differences(initQ)
        print(J)

        JI=np.linalg.pinv(J)
        print("JI", JI)
        
        JI_F = np.array(JI@F, dtype=float).flatten()
        print(np.array(JI_F))

        b = np.linalg.norm(alpha * JI_F, ord=2) #norm of newton step, ƞ
        
        B = np.linalg.norm(JI, ord=2) #norm of jacobian inverse to determine if jacobian itself is nonsingular

        print("b:", b, "B:", B)

        spectral_norm = np.linalg.norm(J, ord=2)
        inv_sn = np.linalg.norm(JI, ord=2)

        cond = spectral_norm * inv_sn

        print("CONDITION NUMBER:", cond) # if the condition number or B is super high, we are at a singularity and we need to move.
        # note that the condition number is always >=1, since it is defined as the greatest singular value of A / smallest singular value of A

        print("SPECTRAL NORM:", spectral_norm)

        h = lipschitz * b * B 
        print("h:", h)

        if h == 1/2:
            p = (1-np.sqrt(1-2*h))*b/h
        elif h < 1/2:
            p = (1+np.sqrt(1-2*h))*b/h
        else: # h > 1/2
            p = None

        return h, p


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

    def central_differences(self, Q, epsilon=None):
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
     
            forward = self.fkin_eval(*(Q + epsilon * I[i]))
            #print(forward)
            backward = self.fkin_eval(*(Q - epsilon * I[i]))
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
        maxiter= 200
        
        J = self.central_differences(currQ)

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

            currError = np.array(F.subs(reps_dof).evalf()).astype(np.float64)
            #print("currError:", currError.flatten())

            if np.linalg.norm(currError) <= tolerance:
                ret = i
                break

            #print(i, end=': ')
            
            newtonStep = (np.linalg.pinv(J) @ currError).flatten()
            #print("newtonStep", newtonStep)
            currQ = currQ - self.alpha * newtonStep
            #print("currQ:", currQ)
            
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
    print(dof3.central_differences([0.,0.,0.]))