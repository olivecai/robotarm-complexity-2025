'''
August 26

Compare how LLS performs when we have random points VS trajectories 

'''


import numpy as np
import sympy as sp
import denavit_hartenberg as dh
import matplotlib.pyplot as plt

P = dh.DHSympyParams() #this lowkey has to be global, sorry
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
                [ t2,  0  ,  0.3, 0 ]
                ]


kinova_dof7_params = [
    [t0,      sp.pi,   0.0,   0.0],
    [t1,      sp.pi/2, 0.0, -(0.1564 + 0.1284)],
    [t2 +sp.pi, sp.pi/2, 0.0, -(0.0054 + 0.0064)],
    [t3 +sp.pi, sp.pi/2, 0.0, -(0.2104 + 0.2104)],
    [t4 +sp.pi, sp.pi/2, 0.0, -(0.0064 + 0.0064)],
    [t5 +sp.pi, sp.pi/2, 0.0, -(0.2084 + 0.1059)],
    [t6 +sp.pi, sp.pi/2, 0.0, 0.0],
    [t7 +sp.pi,    sp.pi, 0.0, -(0.1059 + 0.0615)],
]

dof2 = dh.DenavitHartenbergAnalytic(dof2_params, P)
dof3 = dh.DenavitHartenbergAnalytic(dylan_dof3_params, P)
kinova = dh.DenavitHartenbergAnalytic(kinova_dof7_params, P)
robot = kinova

# print("ROBOT:\n",robot.J_analytic)

cam1 = dh.Camera(0.1,0.05,0,[0,0,4], 4,4, 0, 0) 
cam2 = dh.Camera(-sp.pi/2, 0, 0.5, [0,0,4], 4,4,0,0) #looks at scene from the y axis, world z is cam2 y, world x is cam2 x 
cameras=[cam1, cam2]

vs = dh.DenavitHartenberg_Cameras_Analytic(cameras, robot)
vs.dh_robot.alpha = 1.


kinova_angles = np.deg2rad(np.array([-0.1336059570312672, -28.57940673828129, -179.4915313720703, -147.7, 0.06742369383573531, -57.420898437500036, 89.88030242919922, 0.5]))



def acquire_points_via_random(vs:dh.DenavitHartenberg_Cameras_Analytic, jointranges, Njoints=[], Npoints=[], number_of_points=50, k = 20):
    '''
    add 50 random points to Njoints and Npoints everytime this function is called.

    '''
    joint_configs = [
            np.array([np.random.uniform(low, high) for (low, high) in jointranges])
            for _ in range(number_of_points) #range(2) for a trajectory from point a to b
        ]
    
    print("random generated joint configs:", joint_configs)
    
    # print("randomly generated joint configs:", joint_configs)

    sum_jac_approximation_error=0
    mean_jac_approximation_error=0 # == sum_jac_approxiamtion_error/(number_of_traj_points). this value should be returned.

    
    for p in joint_configs: #includes a and b 

        Njoints.append(p)
        Npoints.append(vs.projected_world_point(vs.dh_robot.fkin_eval(*p)).tolist())
        print("p",p)
        #before we add the point to the cache, we try to estimate its jacobian from the surrounding points.
        approxJ = ret_LLS_jacobian(vs, p, Njoints, Npoints, k)
        trueJ = vs.central_differences_pp(p, vs.projected_world_point([0.,0.,0.])) #the desired projected point does not affect the computation of the jacobian

        print("approxJ\n", approxJ)
        print("trueJ\n", trueJ)

        jac_residual = np.linalg.norm(np.subtract(approxJ, trueJ))
        sum_jac_approximation_error+=jac_residual
        print("jac_res",jac_residual)


    #for every trajectory, get the mean error between the jacobian of each point with the kth 
    mean_jac_approximation_error = sum_jac_approximation_error/len(joint_configs)
    print("mean jac approx error", mean_jac_approximation_error)
    return mean_jac_approximation_error



def acquire_points_via_trajectory(vs:dh.DenavitHartenberg_Cameras_Analytic, jointranges, Njoints=[], Npoints=[], number_of_points=50, k = 20):
    '''
    initialize a random trajectory and as we move through it, cache those joints and points into Njoints and Npoints.
    the error between the true and the approximated jacobian will be very high in the beginning

    count the number of points by pnt_count and by traj_count. (or just get len(Njoints) after.) we will measure the approx vs true jac err by pnt_count to compare with the random generation points method.
    '''
    joint_configs = [
            np.array([np.random.uniform(low, high) for (low, high) in jointranges])
            for _ in range(2) #range(2) for a trajectory from point a to b
        ]
    
    # print("randomly generated joint configs:", joint_configs)

    a = joint_configs[0]
    b = joint_configs[1]

    traj = np.linspace(a,b,number_of_points).tolist()


    sum_jac_approximation_error=0
    mean_jac_approximation_error=0 # == sum_jac_approxiamtion_error/(number_of_traj_points). this value should be returned.

    
    for p in traj: #includes a and b 
        print("p",p)
        #add the point to the cache, then we try to estimate its jacobian from the surrounding points.
        Njoints.append(p)
        Npoints.append(vs.projected_world_point(vs.dh_robot.fkin_eval(*p)).tolist())

        approxJ = ret_LLS_jacobian(vs, p, Njoints, Npoints, k)
        trueJ = vs.central_differences_pp(p, vs.projected_world_point([0.,0.,0.])) #the desired projected point does not affect the computation of the jacobian

        print("approxJ\n", approxJ)
    
        print("trueJ\n", trueJ)

        jac_residual = np.linalg.norm(np.subtract(approxJ, trueJ))
        sum_jac_approximation_error+=jac_residual

        print("jac_res",jac_residual)


    #for every trajectory, get the mean error between the jacobian of each point with the kth 
    mean_jac_approximation_error = sum_jac_approximation_error/len(traj)
    print("mean jac approx error", mean_jac_approximation_error)
    return mean_jac_approximation_error


def ret_LLS_jacobian(vs:dh.DenavitHartenberg_Cameras_Analytic, Q, Njoints, Npoints, k):
    '''
    find the approximated jacobian of joint configuration Q,

    when we have N joints, N points, and can sample from a neighbourhood of k 
    
    return the approximated jacobian
    '''

    approxJ = np.zeros((2*len(vs.cameras), vs.dh_robot.dof))
    if len(Njoints) < k:
        print("Njoints < k, error is norm of true jacobian. (LLS jacobian not computed.)")
        return approxJ
    
    Y = vs.projected_world_point(vs.dh_robot.fkin_eval(*Q))
        
    distances = [np.linalg.norm(np.subtract(Q, storedQ)) for storedQ in Njoints]
    # Get indices of the k smallest distances
    print("this is k:", k)
    nearest_indices = np.argsort(distances)[:k]
    nearest_Q_distances = np.array(distances)[nearest_indices]
    nearest_points = np.array([Npoints[i] for i in nearest_indices])
    nearest_Q = np.array([Njoints[i] for i in nearest_indices])

    dQ = np.vstack([np.subtract(nearby_Q, Q) for nearby_Q in nearest_Q])
           
    dY = np.vstack([np.subtract(nearby_pnt, Y) for nearby_pnt in nearest_points])

    XtX = dQ.T @ dQ

    XtY = dQ.T @ dY

    approxJ = -((np.linalg.pinv(XtX) @ XtY).T) # shape: (error_dim, dof)

    return approxJ

def compare():
    if 0:
        Njoints = []
        Npoints = []
        jointranges = [(-np.pi, np.pi)]*robot.dof
        errors_as_samples_grow_traj = []
        for i in range(150):
            print("random #", i)
            mean_jac_approx_error = acquire_points_via_trajectory(vs, jointranges, Njoints, Npoints, number_of_points=60, k=60)
            errors_as_samples_grow_traj.append(mean_jac_approx_error)

        Njoints = []
        Npoints = []
        errors_as_samples_grow_rand = []
        for i in range(150):
            print("random #", i)
            mean_jac_approx_error = acquire_points_via_random(vs, jointranges, Njoints, Npoints, number_of_points=60, k=60)
            errors_as_samples_grow_rand.append(mean_jac_approx_error)
        print("error_traj:", errors_as_samples_grow_traj)
        print("error_rand:", errors_as_samples_grow_rand)
    if 0:
        error_traj= [1.9297380547432228, 7.094007420156103, 2.032337247686905, 0.8700691111616606, 1.3277979276934864, 0.808825516497519, 0.8433319198093172, 0.8667332590807327, 0.5948135319105463, 0.7296207428656356, 0.9796745789114608, 0.5947621033803174, 0.6071408739923145, 0.961680050379181, 0.6246269769941016, 0.5204673038796528, 0.43610696429635615, 0.5370339522468748, 0.5853261867271041, 0.514152296357968, 0.5043034495862094, 0.5871768782801985, 0.6218467046547806, 0.5007780350035376, 0.6520357761020563, 0.3515432923297249, 0.4987163877395418, 0.3457459643304277, 0.46908678337991766, 0.4624626253949718, 0.5287930052979546, 0.3700085799574176, 0.6901966970334339, 0.6243315858137027, 0.48752210318711225, 0.31475198960500733, 0.5741586488983808, 0.3882762512703915, 0.4483904128472754, 0.32872824849765897, 0.4554134888410576, 0.3433433179229471, 0.49148106173705025, 0.48687588593613634, 0.4525976488650639, 0.4411674234069292, 0.7131379114289557, 0.36708104852639967, 0.539817459339434, 0.9187314132857166, 0.6707604360417619, 0.6177538565378836, 0.5255001355427686, 0.623810786187025, 0.2802560485701552, 0.3999605348409183, 0.48891987314764507, 0.5920728710439678, 0.8239127411779049, 0.4073912933713996, 0.65020404439743, 0.9057662538757631, 0.3767257082661232, 0.5464423576824001, 0.670011407543442, 0.4273387667479021, 0.4079444893740351, 0.46571352728769494, 0.5091610748419841, 0.3323021296194803, 0.35692888611229967, 0.5561773852369565, 0.43111246117235097, 0.34009915416378245, 0.4063027221698273, 0.5834970176856837, 0.2730131058698221, 0.919292376388739, 0.7293176987346541, 0.3319346206682801, 0.4004981771038268, 0.6660057706772118, 0.6977172706877713, 0.6592550706777096, 0.8759187605805788, 0.5637746072515445, 0.3239940851212486, 0.3654911205091181, 0.5279772110789355, 0.9310636968763558, 0.6902044340017846, 0.89936920938373, 0.6816490123266673, 0.41606008636481756, 0.3166633681668445, 0.31738655404207794, 0.7255005297584423, 0.4079059842263321, 0.31035069177218344, 0.700216043355259, 0.25981752076926345, 0.47905424515103207, 0.4821775075924508, 1.137341349494026, 0.6684244686982985, 0.4276095708816789, 0.756047019482122, 0.5998972901168996, 0.3985760303368321, 0.4822204802518215, 0.9078162430289914, 0.6701147221366444, 0.6570126699314592, 0.47595700491099174, 0.3479346830960121, 0.3465183352831231, 0.34254877415282287, 0.6839025039664779, 0.911055976896321, 0.7062903903309486, 0.33394308769516357, 0.5044326108931844, 0.660836660240939, 0.4022231730747199, 0.617631373420716, 0.4715636960384643, 0.4912275577671979, 0.6943808223628414, 0.4896339763098326, 0.4547717558826182, 0.5891235343501325, 0.5678176898729487, 0.3809089590420989, 0.5695724227015705, 0.5475262478672287, 0.258384745698898, 0.44856987343855703, 0.6068543994821874, 0.5461845955255923, 0.3473213435090843, 0.45002396258911065, 0.430771962429393, 0.8695059546186612, 0.49092089244103376, 0.440214909178322, 0.31730443663185304, 0.36997092794331654, 0.5376040217317819, 0.4500558299006433, 0.2788415383734417]
        error_rand= [1.8190853324535305, 1.774399790901301, 1.3922951825914134, 1.103205246282958, 0.9365245299968348, 0.8353678332298304, 0.6877357443684223, 0.6687368430692503, 0.5188771436720879, 0.49461983751726374, 0.4819618709129358, 0.4230014082763826, 0.438491575317059, 0.38515441867852024, 0.4749448063934067, 0.40530124455735766, 0.4109804273773217, 0.3902829997169671, 0.3741472044354569, 0.34066409807230624, 0.36320400639752837, 0.3733469916702207, 0.38806518268250556, 0.3915632005148851, 0.3794871960998641, 0.3786055557234814, 0.378643550259587, 0.427046686512495, 0.3236213429329874, 0.30621331683019676, 0.3695724309020538, 0.41236934615872217, 0.3461816854863605, 0.38801615323268107, 0.39020370169058244, 0.36349666834605665, 0.37289658486892036, 0.3599784958723458, 0.3879414416623978, 0.3518900827516595, 0.3368388525952958, 0.3460102645156615, 0.3208225825379572, 0.3581498717002408, 0.38362183505700964, 0.3898551786636546, 0.3863625054568874, 0.346342412807185, 0.38057217304833757, 0.3326643165201213, 0.3773734939829291, 0.41732609057963993, 0.36084344337989677, 0.36229213789564924, 0.37222975984128015, 0.41736056435118407, 0.3445013943480743, 0.4222785625669107, 0.39492282467888373, 0.3786770781214682, 0.3827338911379553, 0.3930185744693625, 0.41969603300302716, 0.3491564801375754, 0.3778167928254519, 0.35395567616327395, 0.3518852316559297, 0.40655391752463205, 0.39273809986540353, 0.37961504213889596, 0.39177026682966465, 0.33934715833836293, 0.4677205013780939, 0.37957444429905496, 0.4067555817686174, 0.3509219179826505, 0.369165412138895, 0.4121142551389075, 0.38635297180825595, 0.37143495791699277, 0.4073573886923027, 0.40874645737649007, 0.38323827404629696, 0.41877308056815093, 0.3833161406571107, 0.37322715421850633, 0.39404503970740606, 0.38075663277105504, 0.36466059931645517, 0.4008738289444576, 0.4084224522967946, 0.38638531871321724, 0.42206496904115276, 0.41428806934333823, 0.4063477308176851, 0.41588790091222233, 0.3871334818087759, 0.3984061330962513, 0.4050984693617064, 0.3848724335201487, 0.3524368090610143, 0.36854262896559015, 0.3539642006674137, 0.34242301994933055, 0.3972686118564552, 0.38304318910770124, 0.3716634992741517, 0.4251231729893858, 0.4149338754789173, 0.37911011248886184, 0.37814352679198826, 0.421108089956257, 0.39712374442944953, 0.410720305980551, 0.3599072097044104, 0.3944133604495804, 0.4156249466438557, 0.3139620173513496, 0.43878522103073514, 0.3852751206978583, 0.37579905066743635, 0.41364840506282935, 0.3327514723326685, 0.3949635543590373, 0.38084886274721436, 0.45348763789326935, 0.40323489133441104, 0.3810965554297316, 0.3671328682589011, 0.39159685302778197, 0.414837050648293, 0.3592791219342753, 0.40040668520984324, 0.3942802746276655, 0.372730128808121, 0.3783525630018375, 0.44872262408658015, 0.42817702315876016, 0.3859156144390066, 0.3950530996126748, 0.35302989434526083, 0.35573466316372265, 0.4159679178857147, 0.36553042398356816, 0.3897365539526252, 0.4188492877020072, 0.33883242494748644, 0.41816197675347877, 0.44843995485351357, 0.34770460252637403]
        plt.plot(range(len(error_traj)), error_traj, label='random_trajectory', color='red')
        plt.plot(range(len(error_rand)), error_rand, label='random_points', color='blue')
        plt.xlabel("N")
        plt.ylabel("Jacobian Estimation Mean Norm Error")
        plt.title("Measure of Jacobian Estimation Error for Kinova VS N")
        plt.legend()
        plt.show()

def vs_trial1(vs, jointranges, Njoints, Npoints, mode, initQ, desP, n, k):

    robot =  vs

    if mode != 0:
       acquire_points_via_trajectory(vs, jointranges, Njoints, Npoints, number_of_points=n, k=k)

    print("Njoints=",Njoints)
    print("Npoints=", Npoints)
    print("Now attempting visual servoing:")

    currQ = initQ
    tolerance = 1e-3
    maxiter=50

    traj = [currQ]
    errors =[]

    desPP = robot.projected_world_point(desP)

    if not mode:
        J = robot.central_differences_pp(currQ, desPP)  
    else:
        J =  ret_LLS_jacobian(vs, currQ, Njoints, Npoints, k)

    alpha = 0.1

    for i in range(maxiter):
        print("###################### i:", i)
        currError = robot.projected_errfn_eval(currQ, desPP) #desired-current

        if np.linalg.norm(currError) <= tolerance:
            break

        Jinv = np.linalg.pinv(J)

        corrQ = (Jinv @ currError).flatten()
        print(f"currQ: {currQ}\ncorrQ: {corrQ}\ncurrError: {currError}")


        currQ = currQ - alpha * corrQ

        # for i in range(len(currQ)):
        #     if currQ[i] < jointranges[i][0] or currQ[i] > jointranges[i][1]:
        #         print("out of bounds")
        #         print(jointranges[i])
        #         print(currQ[i])
        #         print(currQ)
        #         return 0
            
        if not mode:
            J = robot.central_differences_pp(currQ, desPP)  
        elif mode >= 2: #skip jac update for 1
            J =  ret_LLS_jacobian(vs, currQ, Njoints, Npoints, k)

            print("approx:",J)
            print("true:", robot.central_differences_pp(currQ, desPP)  )

        traj.append(currQ)
        errors.append(np.linalg.norm(currError))

    traj = np.array(traj)
    if 0: 
        robot.dh_robot.rtb_robot.plot(traj, block=True)

    print(f"n:{n}, k:{k}, initQ: {initQ}, desP: {desP}, mode: {mode}\nerror from goal during visual servoing:\n{errors}")
    print(f"real world final position (just curious) {(robot.dh_robot.fkin_eval(*currQ))}")

    return errors


jointranges = [(-np.pi/2, np.pi/2),(-np.pi/2, np.pi/2),(-3*np.pi/2, 3*np.pi/2),(-np.pi, np.pi),(-np.pi/2, np.pi/2),(-np.pi, np.pi), (-3*np.pi/2, 3*np.pi/2),(-np.pi/2, np.pi/2)]
# jointranges = [(-np.pi/2,np.pi/2)]*robot.dof
initQ = np.deg2rad(np.array([-0.1336059570312672, -28.57940673828129, -179.4915313720703, -147.7, 0.06742369383573531, -57.420898437500036, 89.88030242919922, 0.5]))
desP = [0.3,-0.2,-0.0]
# initQ=[0.0,1.0,0.9]
# desP=[0.2,0.3,0.1]

Npoints=[]
Njoints=[]
N_range=range(10)
jac_errs=[]
for N in N_range:
    jac_err = acquire_points_via_trajectory(vs, jointranges, Njoints, Npoints, 2,200)
    jac_errs.append(jac_err)
print("jacobian errors per trajectory:", jac_errs)

print("Npoints=", Npoints)
print("Njoints=", Njoints)

vs_trial_error_trajectories_for_different_N=[] #list of lists
vs_trial1(vs, jointranges, Njoints, Npoints, 0, initQ, desP, None,None)

N_range=range(10)
for N in N_range:
    error=vs_trial1(vs, jointranges, Njoints, Npoints, 2, initQ, desP, 20,200)
    print("N:", N,"\nERRORS:", error)
    vs_trial_error_trajectories_for_different_N.append(error)

for errors in vs_trial_error_trajectories_for_different_N:
    try:
        plt.close('all')
        plt.plot(range(len(errors)), errors, label='kinova', color='red')
        plt.xlabel("iters")
        plt.ylabel("Jacobian Estimation Mean Norm Error")
        plt.title("Measure of Jacobian Estimation Error for Robot System VS N")

        plt.show()
    except:
        print("PLOT IS NONE")
        pass

