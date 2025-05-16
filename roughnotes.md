roughnotes.txt

# cde chpt 5
polynomial interpolation  
taylor polynomials  
Consider an alternative to taylor series: interpolation nodes, define polynomial interpolant that interpolates function at the nodes.  the nodes are … points of the function within the range [a,b] ?  
Is the error stuff for linear interpolation and the slopes relevant?

### 5.2.3 comparing accuracy:  

> the linear interpolant is asympotically more accurate than a constant interpolant and that the piecewise constant approximation is first order accurate while the piecewise lienar approximation is second order accurate. 
 
 *what does it meant to be nth order accurate?*  
 ^ First order accurate means the error assoc with the approx reduces LINEARLY with step size, so halving the step size halves the error. So, for second order accurate, halving the step size will quarter the error. The higher the order of accuracy, the better.  


> increasing the degree of the interpolant increases the order of accuracy.

*what is the interpolant?*  
interpolant: the function used to estimate or interpolate values of another function at point between known data points.  
extrapolant is when we extrapolate to datapoints outside of range.

higher degree interpolants are generally less asymptotically accurate.

### 5.3 vector spaces of piecewise polynomials

instead of increasing the order of the interpolating polynomial on given interval; use PIECEWISE POLYNOMIAL APPROX on smaller and smaller partitions.

partition of interval and its corresponding mesh function

**hat  function**  
- hat function assoc to each node
- hat functions are nodal basis

### 5.4.2 piecewise linear approximation

using piecewise functions to interpolate function.

*how are the nodes chosen?*

### 5.4.3 adapted meshes

> suppose f(x) is a given function on an interval I and we want to compute a piecewise polynomial interpolant of a specified degree st the error in a specified norm is less than a given tolerance.

*what does "specified norm" mean?*

*how to distribute points efficiently to use the least number of points to achieve the desired accuracy?*

answer:

> the mesh with the minimal number of mesh points is a mesh that equidistributes the error st the contribution of total error from each sub-interval of the mesh is roughly equal.

let maximum error over each sub-interval == given tolerance.

equidistribution of error!!!

### 5.4.7 approximation by trigonometric polynomials

if f is period 2pi (which ours should be), f is continuous and f' is continuous except at a finite number of points in a period.... then f can be represented as a convergent Fourier series! This corresponds to the quadratic approx in the space 1, cosx, sinx, .. cosqx, sinqx

# cde chpt 6: galerkin's method

galerkin's method: seek an approx solution in a finite dim space spanned by a set of basis functions which are easy to differentiate and integrate, together with an orthogonality condition determining coefficients or coords in the given basis

> seeking a solution of a DE as a linear combination of simpler basis functions is old. newton and lagrange used power series with global polunomials and fourier and riemann used fourier series based on tirgonometric polynmials. 


when the matrix is ill conditioned ie we approach singularities, we MUST use an orthogonal basis OR use piecewise polynomials where the basis function are nearly orthogonal.


to use galerkin's model with piecewise polynomials, we need to model our system as a DE. 

youtube video on galerkin's video:
- orthogonal projection of error. dot product of error and basis vector e1  (or instead of basis vectors, think basis functions!) function is = 0. 
- EIWxxxx - f = 0
- galerkin is strong form! unlike ritz, which is weak form.
- in family of weighted residual method
- assume apprx diplaement, sub that into equaion, that prodcues residual error, make sure error is orth to bassi functions. ORTHOGONAL PROJ OF ERROR, the galerkin method is two steps: 1. assume apprx diplacement function. 2. Sub that approx diplacement fn into equation, then eNFORCE the error is orth to basis functions.



### discussion

i want to optimize these mesh points t the error is in some tolerance: we can try doing this because we dont have a closed form for the function. brute force search on some granularity: the most naive way to pptimize is discretize and try every combination - throw it at some prebuilt optimization package and see if you can gradient descent 

what would it mena? if you optimize for where/how many points to get this meshof correctness, you need to be able to access the function at all of those points - what do you want to discuss withh it? what are you trying to say? in the practical case, we are doing visual servoing but you wont have that level of contorl over where you can sample the function - you could in advance sample the function everywhere and then do it but thats not very practical. if you run VS and geenrate a function approx as you do it, what is ther error as you do it 

neural network - function approx - train off of some loss function - how bad is your prediction
whenever you do neural network, you are just tryign to approximate 

sounds like you are tyring to approximate how amny jacobian updates you will need

predict the linearity by predicting how many updates you will need

if youre just plugging into a function your LOSS, and then tryignn to predict the linearity - probably will not be useful

you can try to learna  network wwhere you recurrently input error and motion over time and thrn predict how likely you are to need an update at that moment. But then you feed it information that might gewneralize - consider that NN must be generalizable. Predict how likely you are to need a NN, input info st the network can learn about the function itself you are dealing with so that you get generalizaboility. IF you get the function to learn in some way abotu the function so that you can give it a new function. 

instead of focusing on voncegren non oconvergece, look at the function in general - like plotting what does the error function look like over the entire joint space - minimize the error is ultimate goal - the complexity of that gives you idea of how complex the optimization ladnscape is - when you have convex function, happy because optimzation landscape is easy to optimize, you have same thing: you can look at how difficult of an optimization function am i dealing with - are you likely to have good convergence or bad convergence. if you have a function easy to optimize, likely to have good convergence.

convergence function probably not as useful:
- gives sense of HOW LOCAL the solution method is
- if function is very complex, probably very local convergence radius
- but you get more of the story if tyou look at the whole function
- dont looka t the convergence radius

probably better analysis by just looking at visual motor function 

two error bounds:
- CDE pg 308
- error from approx to real
- error between approx grad and true grad

GOAL:  
- how dodes complexity change as little things as cnagef
- how does the compelxitt of thisd change as the degrees of freedom change (HARD TO VISUALIZE) 
- i can analyze how this changes when i add another jtn even tho ic ant see it - plot the error functions? discretize grid over jtn space, look at what is the step in each grid element, for ever 00.1 rad you have a grid element, if you have 0.1, what is the error you are getting. if i do 0.01, what theoretical error are you getting in your error bound? 
- look at the optimization - you might be able to - lets say do optimziation over best error an there are a bunch of points in one simialr space, then aorund thesejioint angles, you canmaybe it is very nonlinear and you need a loit of sections - optimization might be the only way to easily find wat those error are, since over a grid you might skip over details OR if you do 0.1 its just reallybad since most of the funciton is nonlinear and you have high error in a lot of areas, grid isnt very informative. 
- ENDGOAL: mgiht be some way to come up with predictor- given these erro constraint and this robot/DOF/# cameras, then output some measure of how linear the function is - there might be some way att hte ned to make a prediction of - this is likely to be a very difficult problem, you might get VERY local convergence, might get a ton of reupdates. 
- look at interpolation with higher degree function, ie L4 L5. 

13.13.1 there is general result of error 

# May 15 2025

Today I drank a huge coffee and I spent a long time coding slices in my 3D scatterplot, but it might not even be super useful. It turns out I was focusing a lot on the convergence function, but I should be focusing on the actual underlying function itself. 

TODO: 
- CDE chpt 13
- make a linear approximation of my function in real time? 

- can we try a similar thing as cole but when we come across a new point, instead of 

q -> e 

for every output in e, triangulate q 

### CDE chpt 7 solving linear algebraic systems

solving Ax=b, where usually x=A-1b will suffice. However, A may be ill defined, AND A-1 is never the most efficient way to compute the solution. 

NOTE: try to give up on the idea of measuring the error for now. What if we can just DESCRIBE the position itself, not the accuracy? Measure simply the Jacobian at the point? Dont think about the error. 

Jacobi method and Gauss-Seidal method. Gauess Seidal is a little faster but can be harder to implement.

Can we implement Gauss-Seidal for nonlinear multidimensions? Looks like we need:
- matrix to describe our system
- the true function so that we can compare our current approx 
- residual tolerance for each step

### CDE chpt 8 two point boundary value problems

*what is a two point boundary value problem?*  
answer:  
in IVP, conditions are given at a single point. In BVP, conditions are usually given at the endpoints of the interval

8.2.3 looks at adaptive error control: 

NOTE: so, we could try to formulate the differential equations, OR we could do real time linear interpolation.

### chpt 9 scalar initial value problems

stability property:

>  If we think of numerical discretization as introducing a kind of perturbation of the true solution, then the stability properties of an initial value problem influence the error in a numerical solution of an initial value problem. In general, we expect the error of a numerical solution to grow with time, since the error at one time is affected by the accumulation of errors made at earlier times.

dicont and cont galerkins method

### chpt 10 IVP for systems

autonomous and nonautonomous problems - nonaut is usually richer and easier 

how stable is a solution? ie how sensitive is the solution to perturbations in the data?

lots of examples, and motion of satellite. on the lookout for how to model our robot: function of q0, q1, etc. output our location and jacobian... ???

### chpt 11 calculus of variations

find a function u(x) that minimizes the integral F(v)=integral from 0 to 1 of f(v,v')dx
overall all function v(x) where v(0)=u0 and v(1)=u1 where f(v,w) is a two var func, and u0,u1 are our boundary conds.
F(v) is the Lagrangian of the func v

the brachistochrone problem - find the shape of a piece of wire conendcting two points st a bead slides down the wire under the force of gravity in the least amt of time. this is a minimization problem. 

NOTE: look into the brachistone problem using successive refinement involvign piecewise linear functions

what is the lagrangian system

### chpt 12 computational mathematical modeling

adaptive methods
 - stopping criterion 
 - modification strategy in case the stoping criteria is not satisfied

### chpt 13 piecewise polynomials in several dimensions

tent functions, mesh,

look into Dilaunay 

Looked into the differential equations to try to use the CDE methods, but to even obtain the differential equation, we need to get a lot of information about the robot: at that point, just calibrate your camera. It is instead advantageous to try to create the linear mesh from just visual servoing or real-time linearization.

The well known generalized dynamical equation of a robot is I(q)q'' + h(q',q)+f(q) = u 
But to even know this, is onerous!!!!!!!!!!!!

So, let's try real time triangulation of the mesh: but how can we traverse the function if not inverse kinematics?
Step 1: Choose a specific grid/step size
Step 2: go back and REFINE !!!! 

TODO: make a very sparse grid and go through each triangle and see if you need to make it more complex.
1. download triangulation software
2. make a linspace and get the position value for certain datapoints (seperate x and y plots, each plot is q0 by q1)
3. get the overall position error for each triangle: if overall error is greater than tolerance, we should SUBDIVIDE. We know the actual function (for now) since we can just compute the forward kinematics. Should this method be recursive? For each segment, subdivide until error is lower than RESTOL

Here are the recursive steps:
1. For a very sparse linspace of params (ie q0,q1 for now) plot the position PERCIEVED (so if we are in visual servoing, just plot the perceived position.) 
2. The recurrence step occurs for each triangle: move to (? where/how should we subdivide the triangle?) and measure the error between that specific point and the actual function. if error is > RESTOL, then create a point to add to the triangles mesh there. 
3. I think it's best if the triangluated point is in the centroid of the triangle (it will form weirdly if we take the border lines), though it's certainly a lot of recursions. Formula for the centroid, G, of a triangle in R-n is G = ( xa1+xa2+xa3 / 3, xb1+xb2+xb3 /3, ... xn1+xn2+xn3/3) So it's generalizable to higher deg DOF

This is fine, but how can we make a piecewise linear approximation WITHOUT referencing the actual function? 
In other words, we can't use the idea of error. Can we use the error of our own approximation? 

TODO afterwards: successive mesh refinement may not work for a robot to find in real time, since it is both innefficient and not always feasible that we can nump around everywhere. What if instead we try somethign like successive mesh refinement?