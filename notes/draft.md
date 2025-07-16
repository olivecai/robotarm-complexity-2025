# Draft

#### Jul 16 2025

## Online inverse kinematics with minimal Jacobian updates:
- Real time evaluation of the radius of convergence of our initial position:
- If we cannot converge with 1 jacobian, move to the edge of that convergence ball and evaluate again

CONS: we do not have access to the global Lipschitz constant

## Calibrate beforehand to obtain: local and global Lipschitz constant and Jacobian:
- allows us to calibrate everything once and never need to calibrate again.
- Have N planes/simplices to assign over joint space. Sample sparsely over joint space and detect where the greatest differences are to sample over the space again.
- Lipschitz constant should be overestimated.

CONS: potentially very resource intensive, need to determine which simplex we are in during the robot movement, if we sample over the space anyway, we can ask, why not simply obtain the joint position anyway?

## Ideal:

If we could approximate the global lipschitz constant, we could solve this problem.