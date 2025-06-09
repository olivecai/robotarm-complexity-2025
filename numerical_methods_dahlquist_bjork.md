### June 6

# Chapter 1.1

## 1.1.1 fixed point iteration

> We see that if x0 is chosen sufficeintly close to the root a, the iteration will converge if |F'(a)| < 1. In this case, a is the POINT OF ATTRACTION. 

Converge is faster the smaller this value (which is generalized to the spectral radius in higher dimensions) is. If This value is > 1, then a is a POINT OF REPULSION and the iteration diverges. (page 3)

There is monotonic and oscillatory convergence. 

NOTE: the derivative of the solution itself is taken, not the residual step. It is more to do with the point being a point of attraction in the system, than how it relates to the overall function itself.

FIXED POINT FORM is not necessarily the equation you may take: toggling it affects convergence...

*How can we make the iteration equation more robust for visual servoing? Ie use a different kind of iteration other than NEwton's method? Ie modify newtons method so that it works for a broader range of values? We still have the issue of the spectral radius below <1 yet converging.... How can we understand this?*

## 1.1.2 newton's method

F(x) = x+ k * f(x)

So, if F(x)=x, then f(x) must = 0.

F'(x) = 1 + k*f'(x) and for a certain point a,
F'(a) = 1 + k*f'(a). Since we would like F'(a) == 0, we let k = -1/f'(a), which cancels out the f'(a) to -1, and 1-1==0

**equation for the tangent:**

we know the tangent equation is y - f(x0) = f'(x0) * (x - x0)
and by setting y=0 we obtain the approximation x1=x0-f(x0)/f'(x0).
when y=0, we can solve for the roots!

> Newton's method can be generalized to yield a quadratically convergent method for solving a system of nonlinear equations

## 1.1.3 trapezoidal rule

numerical integation and reducing area - uses local approx of the integrand with a polynomial of higher degree

## 1.1.4 central differences

> forward difference approximation is exact only for a linear function and it is only first order accurate in the general case.  the centered difference approximation is exact also for a quadratic function, and is second order accurate in the general case.

# Chapter 1.3 Matrix Computations

A = LU (lower and upper diagonal matrices) and A^-1 = U^-1 * L^-1

CHOELSKY FACTORIZATION:
- if A is a symmetric positive definite matrix, there exists the unique factorization A = R^T * R

SPARSE MATRIX:
- iterative methods can be used on sparse matrices instead of Gaussian elimination, taking advantage of the number of zeroes present

# Chapter 1.4 Linear Least Squares Problem

SVD 

Moore Penrose Inverse - four penrose conditions:
1. AXA=A
2. XAX = X
3. (AX)^T = AX
4. (XA)^T = XA

The Numerical Rank of a Matrix:
> It can be shown that perturbations of an element of a matrix A result in perturbations of the same, or smaller, magnitude in its singular values.

Consider the matrix A that we perturb by E. Each singular value of the perturbed matrix A+E can change by at most the spectral norm of E. So if E is small in norm, the singualr values will be stable.

NUMERICAL RANK refers to how many singular values are above a small threshold

# Chapter 4 the Interpolation Problem

find a polynomial p(x) such that p(xi)=f(xi), i=1:n

runges phenomenon --> oscillating of polynomial near roots. minimize runge's phenomenon by sampling with chebyshev nodes

> the Lagrange interpolating polynomial is the unique polynomial of lowest degree that interpolates a given set of data.

Lagrange polynomials: a set of orthogonal polynomials

Bezier curves are used in typography... wow! very cool

splines, curves

# Chapter 6 Solving Scalar Nonlinear Equations 

BISECTION method:
- uses the intermediate value theorem to infer if an interval [a,b] contains at least one root of f(x)=0.
- convergence is rather slow but independent of the regularity of f(x). (methods like Newton's method are much faster and assume regularity of f(x))
- does not tell us how many roots there are

NOTE: Is there a way to use the bisection method to guarentee convergence?

there is also the MULTISECTION method, which is better for finding several roots in the interval, since subintervals can be processed in parallel.

TERMINATION CRITERIA
- crude way: set max iters

for iterative methods with fast convergence, stop when for the first time the approximation xn satisfies the two conditions:
1. |x(n+1) - xn| >= |xn - x(n-1)| (that is, our step seems to have gotten larger all of a sudden)
2. | xn - x(n-1)| <= tolerance

## 6.1.4 fixed point iteration

let phi be a continuous function and {xn} the sequence generates by x(n+1) = phi{xn}, n=0,1,2,.... for some initial value x0.
assuming that lim n --> infinity of xn = a, so lim n --> infinity of phi(xn) = phi(a)

a is the fixed point of the mapping x --> phi(x)

phi(x) = x - f(x)*g(x)

where:
- x=phi(x)
- solve for f(x) = 0 
- g(x) is any function such that g(a) != 0 

if lim n-->infinity = a for all x0 in a sufficiently close neighbourhood of a, then a is a point of attractyion, otherwise is a point of repulsion. the case where the spectral radius (or, in other qords, the first derivative of the desired point ) is ==1 is indeterminare and may or may not converge. 

the existence of a fixed point is not assumed a priori 

*Newton's Method VS General Fixed-Point Form*

What is the difference between x(n+1) = xn - f(xn)/f'(xn) (newtons method) and phi(x) = x-f(x)*g(x) (general fixed point form)

well, seeing them as iterations:
newton's is x(n+1) = xn - f(xn)/f'(xn)
general is x(n+1) = xn - f(xn)*g(xn)

Newton's method is a specific case of general fixed point form. 

here phi(x) is the fucntion that transforms one guess to the next, the 'update rule'.
so, we can define x(n+1) to be phi(xn). 

g(x) = -1/f'(x) in newton's method

*Great. In our fixed point inverse kinematics for the robot arm, we are using Newton's method by using the AJcobian, but it's possible we can try the Hessian or soem other matrix. Either way, we have a function f(x) the LOSS

Linear convergence is typical of fixed point iteration.

Newton Raphson is x(i+1)=xi-f(xi)/f'(xi)
Newton method is x(i+1)=xi-f'(xi)/f''(xi)


Look into the "particularly elegant combination of bisection and the secant method" devceloped in the 1960s by van Wijngaarden, Dekker, and others at the Mathematical Center in Amsterdam 


# Volume II

# Chapter 11 Nonlinear Systems and Optimization

We should just try any kind of way for global convergence methods, and see if that gives better performance.

Something that is still very much a loose thread is the fact that the spectrla radius being < or > 1 is inconsistent with our actual results, maybe we should get rid of dmaping and see if it lines up. Sigh!!!!!!!!!!!!!!!

