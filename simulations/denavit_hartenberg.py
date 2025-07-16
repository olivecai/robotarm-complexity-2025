'''
July 15

This program obtains analytic expressions from the Denavit Hartenberg parameters.


'''

import sympy as sp


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
        self.cartvars= symbolclass.cart_space_vars
        self.taskvars = symbolclass.task_space_vars
        self.jntvars = symbolclass.joint_vars
        #dh_params is a double nested list, where each row is one joint.
        transforms=[]
        for param in dh_params: # for each joint 
            transforms.append(self.transformation_matrix_DH(*param)) #transformation_matrix_DH returns the DH matrix for that single joint...
        ee_matrix = sp.eye(4)
        for i in range(len(transforms)): #chain multiply the DH matrices to get the final position
            ee_matrix *= transforms[i]

        ee_translation = ee_matrix[:,3] #translation final position 
        self.ee_matrix = ee_matrix #entire matrix including rotations etc
        self.ee_translation = sp.Matrix(ee_translation)
        self.dof = len(dh_params) #degree of freedom correlates to the number of dh parameter rows

        print("ee_translation:", self.ee_translation[:3])
        self.F = sp.Matrix(self.ee_translation[:3]) - sp.Matrix(self.cartvars)
        print("F:", self.F)
        
        self.J = self.F.jacobian(self.jntvars[:self.dof])



    def calc_kantorovich_vars(self):
        #error function F...
    
        #initial newton error step...
        #F jacobian inverse @ F <= eta
        self.B = self.J.inv()
        BF = self.B*self.F
        BFTBF = BF.T*BF
        print("BFTBF", BFTBF)
        BFTBF_evals = BFTBF.eigenvals()
        largest_eval=sp.Max(*BFTBF_evals.keys())
        self.b = sp.sqrt(largest_eval)
        print("J", self.J)
        print("B:", self.B)
        print("b:", self.b)

        #global lipschitz...
        #global lipschitz == the maximum absolute value of the singular values of the jacobian... in other words the supremum spectral norm of J
        JTJ = self.J.T * self.J
        JTJ_evals = JTJ.eigenvals()
        largest_eval = sp.Max(*JTJ_evals.keys())
        lipschitz = sp.sqrt(largest_eval)
        # we are bounded so technically we should be able to compute the analytic lipschitz...
        print("lipschitz:", lipschitz) 

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