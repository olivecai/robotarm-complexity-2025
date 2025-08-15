'''
August 15 2025

Can we make an online mesh for the robot?

Have a desired position in a well conditioned joint pose.

Generate random joint positions as our to-be-start points.

For every generated start point, perform Newton Policy and keep track of where the Jacobian update was needed by pushing it to a list.
During each Newton Policy, it's possible two trajectories will need to update near the same Jacobian, so for every Jacobian update, first verify that this point (within some tolerance) is not already in the mesh.

Should we make a mesh of simplices, or should we define 'hotspot' centers? For now let's see how the mesh of simplices looks.

What we will need:
- A list of points to send to scipy Delaunay in the end.
- The lipschitz constant of f'

'''
