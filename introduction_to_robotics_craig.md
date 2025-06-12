# Introduction to Robotics Craig

## 1.2 the Mechanics and Control of Mechanical Manipulators

coordinate system or frame rigidly attached to object

revolute/rotary - displaced by joint angles
prismatic joints - joint offset

manipulators consist of nearly rigid links, 
DOF - indep pos vars

end effector position is generally described by giving description of the TOOL FRAME, which is attached to the end effector, relative to the BASE FRAME

joint space vs cartesian space

Jacobian specifies a mapping from velocities in joint space to velocities in Cartesian space.
At singularities, Jacobian is not invertible.

Singularity ==> robot becomes locally degenerate


