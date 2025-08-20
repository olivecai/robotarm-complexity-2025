'''
August 18 2025

Jacobian policy that:
- initializes the central differences Jacobian as the first Jacobian
- performs the Broyden update on every iteration
At the same time, that Broyden's update will only get us so far.
So, we should also be evaluating whether it is time to refresh with the Jacobian update .

'''