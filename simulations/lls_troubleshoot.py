
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
robot = dof2

# print("ROBOT:\n",robot.J_analytic)

cam1 = dh.Camera(0,0,0,[0,0,2], 2,2, 0, 0) #looks down directly at the scene, basically replicates the scene
cam2 = dh.Camera(-sp.pi/2, 0, 0, [0,0,2], 2,2,0,0) #looks at scene from the y axis, world z is cam2 y, world x is cam2 x 
cameras=[cam1]

vs = dh.DenavitHartenberg_Cameras_Analytic(cameras, robot)
vs.dh_robot.alpha = 1.
k=10

Q = [0.5, 0.]
inputs=[]
outputs=[]
for _ in range(k):
    perturb = np.random.uniform(-0.1,0.1, size=[len(Q)])
    print(perturb)
    perturbed = np.add(Q, perturb)
    inputs.append(perturbed)
    output = vs.projected_world_point(vs.dh_robot.fkin_eval(*perturbed))
    outputs.append(output)
    print(perturbed, output)

inputs = np.array(inputs)
outputs = np.array(outputs)

print(inputs)
print(outputs)

Y = vs.projected_world_point(vs.dh_robot.fkin_eval(*Q))

dQ = np.zeros((k * k, vs.dh_robot.dof))
dY = np.zeros((k * k, Y.shape[0]))
for i in range(k):
    for j in range(k):


        dQ[i * k + j, :] = inputs[i, :] - inputs[j, :]
        dY[i * k + j, :] = outputs[i, :] - outputs[j, :]

print("dQ:",dQ)
print("dY:",dY)

XtX = dQ.T @ dQ

XtY = dQ.T @ dY

approxJ = ((np.linalg.pinv(XtX) @ XtY).T) # shape: (error_dim, dof)

print(approxJ)

print(-vs.central_differences_pp(Q, Y))

print("################")


Y = vs.projected_world_point(vs.dh_robot.fkin_eval(*Q))
distances = [np.linalg.norm(np.subtract(Q, storedQ)) for storedQ in inputs]
# Get indices of the k smallest distances

nearest_indices = np.argsort(distances)[:k]
nearest_Q_distances = np.array(distances)[nearest_indices]

nearest_points = np.array([outputs[i] for i in nearest_indices])
nearest_Q = np.array([inputs[i] for i in nearest_indices])

weights = [i**-1 for i in nearest_Q_distances]
sum_distances = np.sum(weights)
weights = weights/sum_distances

# dQ = np.zeros((k * k, self.vs.dh_robot.dof))
# dY = np.zeros((k * k, Y.shape[0]))
# for i in range(k):
#     for j in range(k):

#         dQ[i * k + j, :] = nearest_Q[i, :] - nearest_Q[j, :]
#         dY[i * k + j, :] = nearest_points[i, :] - nearest_points[j, :]


dQ = np.vstack([np.subtract(nearby_Q, Q) for nearby_Q in nearest_Q])
dQ = np.vstack([w * dq for w, dq in zip(weights, dQ)])

dY = np.vstack([np.subtract(nearby_pnt, Y) for nearby_pnt in nearest_points])
dY = np.vstack([w * dy for w, dy in zip(weights, dY)])

# W = np.diag(weights)

# print(f"Q: {Q}")
# print(f"nearestQ: {nearest_Q}")

# print(f"Y: {Y}")
# print(f"nearest Y: {nearest_points}")
# print(f"dQ: {dQ}")
# print(f"dY: {dY}")

XtX = dQ.T @ dQ

# print("XtX", XtX)

XtY = dQ.T @ dY

# print("XtY", XtY)

approxJ = ((np.linalg.pinv(XtX) @ XtY).T) # shape: (error_dim, dof)

# #2) robust solve via lstsq (J^T is solution)
# Jt, *_ = np.linalg.lstsq(dQ, dY, rcond=None)
# approxJ = Jt.T  # (m, dof)

print(approxJ)

print(-vs.central_differences_pp(Q, Y))