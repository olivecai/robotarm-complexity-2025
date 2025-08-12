'''
Aug 11 2025

The truncation error in the Taylor series for central differences jacobian is O(h^2).
So, if we have h<1, it's likely that the error will be VERY SMALL.
Specifically the truncation error of the Jacobian is (f(3)(x) * h^2 / 6) + O(h^4), where f(3) is the third derivative of the function.

We dont know the third derivative of the function, but we can assume that it is bounded by some constant M.
How to get M?
'''