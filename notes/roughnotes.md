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

# May 23 2025

Is there a way to count the number of points in a region, kind of like taking the integral of our linear model, to quantify how linear different segments are? Would we have to count triangles instead?

# May 26 2025

We are successfully plotting the meshes of the n-DOF robot arm positions. Now we can try to quantify the nonlinearity by counting the number of points in each concentrated region..... OR what if we make a note of areas that experience lots of recursion? 

We have a few problems:

- Is the linear mesh correct? it looks correct for the 1DOF and 2DOF, but is there a way to validate for higher DOF? Answer: look into Hessian
- Is there an algorithmic/smart way we can quantify areas of nonlinearity? Ie, we know we want to count the number of points in a region, but how can we algorithmically find those regions? Should we keep track of these regions through recursion instead?

After we identify these nonlinear regions, can we create a 'smart' inverse kin method that only updates the Jacobian as needed? Ie, give certain regions a 'rank' which corresponds to how nonlinear it is, and if our inverse kin method is entering that region, we can update?

*What can we do with this linear mesh?*
1. Make a program that, given our linear piecewise function, creates 'regions of linearity' such that moving from region A to B corresponds with a Jacobian update, since the two regions are associated with different linearity. 
2. Experiment with different kinds of Jacobian updates: Broyden's, Secant, Central Diff, etc

We are using an offline approach to create the global function. Can we use an online approach to create the global function? 

Make a mesh that knows its nonlinear points: 

Inverse kin: update real EE and move into a new spot. If prev position was in region A and new position is in region B, then update the Jacobian. If successful convergence, then return 1. If fail, ret 0.

Right now there is a certain trouble in creating regions to indicate a Broyden update is needed, since we are dealing with a discrete mesh, we may need to repeatedly take each point and compare it to the mesh to see which region our point lies in...

# May 27 2025

Use barycentric coordinates to calculate whether or not a point is inside our shape. 

*How can we keep track of which triangle we are currently in?*

Firstly, WHICH triangles?
- The manually calculated ones for the centroid, OR the delaunay mesh triangles? PROS: the manually calculated ones already exist AFAIK but how can i access and query triangles/tetrahedrons created by the delaunay mesh?.... There are some complications in the fact that TOO MANY shapes are being formed right now. Figure out how to REDUCE and ACCESS the number of triangles/tetrahedrons in the Delaunay mesh.

Secondly, how can we KEEP TRACK of our current triangle and update it? Answer: iterate through every triangle, save the points of the triangle as A, B, and C, and then keep VALIDATING (not iterating) and then when validate==False, THEN we can iterate/query for whichever new triangle/tetrahedron we are in....

It turns out that scipy.spatial.Delaunay has something called 'calculate simplex'.
Yay!

TODO:
- debug: why does find simplex ret -1 for many of our points? This should not be the case, so something is mysterious and WRONG...

newtons method - have some point, take the tangent and intersect the point with x acis and that is new location. . problem when tangent slope=0

# May 28

### Notes from Dylan:
do same computationw ith analytic jac and compare.

analyze analytic jac and given its properties, will we converge? rate of convergence

newtons method,
geometric multiplicity - convergence rate is linear instead of quadratic
in general case, error for each point is QUADRATIC
when geo mult, error is linear (constant) 
higher order error convergence is better than linear - you want error to drop faster

look up newton method - heith, 

e-vals of jac and they are same, 

if largest e-val is below 1, then CONVERGES. (spectral radius fixed point method)

how singular? 1 very large eval and 1 small - transofrming in just one direction 

veer away from numeric

if we have the equations, we have a lot of info - most of the stuff is known - we may not know exact cam params, but we know robot jac, fkin, etc. parts of it - camera that we dont know 
space is alr ish well known - onyl certain things unknown

select init points in grid: uniform grid - other strategies for meshing: 
- Chebyshev points, Runge's  (maybe make more sense, since circular)

lean on sym lang 

### Notes:

Chebyshev nodes - nodes for polynomial interpolation and numerical integration- projection of a set of equispaced points on unit circle onto its diameter.

When using polynomial 

Witch of Agnesi

### Chebychev's Nodes:



### Runge's Phenomenon:

When we interpolate a function with equidistant nodes, the resulting interpolation oscillates toward the end of the interval, because of two things:
- the magnitude of nth order derivatives grow quickly when n increases
- equidistance b/n points leads to a Lebesgue const that increases quickly when n increases

### Lagrange polynomial:

unique polynomial of LOWEST degree that interpolates a given set of data

susceptible to Runge's phenomenon of large oscillation

### Unconstrained Optimization Strategies
https://www.sciencedirect.com/science/article/pii/S0096300308005985#:~:text=They%20have%20been%20intensively%20studied,to%20find%20a%20step%20length%20:

- line search
- trust region
- interval of uncertainty 

- new robust line search technique based on Chebyshev polynomials 
> Newton’s iteration converges q-quadratically to alpha under certain assumptions.

*So, how can I sample my mesh at Chebyshev points instead?*

TODO:

- download symbolic toolkit and analyze Jacobian, look into eigenvalues, probably should read other papers

# May 29

So far, simplices update beats non-simplices everytime, reducing total error.
It seems the equidistant mesh beats chebyshev often, especially when maxiter <= 100.

So, when we plot the trajectory, we can see how nonlinear it is: the number of nonlinear segments should hopefully correlate with the number of Jacobian updates we do (??? is this true?) 

We need to understand better what the trajectory path means!

### Notes from Dylan:

SE3 point - we know EE in RW, we know camera proj of 3D point to 2D,

calib - includes camera position in 3D space (external), focal len & center, etc (internal)

routines to calib

value in uncalib - just put any camera anywhere, more robust

TODO: 
1. analytic jacobian - compare trajectories and see how our simplices method compares
2. plot the arm - oscilates?
3. for each simplex, can we designate a predetermined Jacobian? This will save us time later so that we don't need to do central differences with the real robot :3
4. higher DOF
5. camera 

Note: to follow up on 3., we can do this relatively easily by perhaps calculating the analytic Jacobian for the centroid of each simplex, and assigning that Jacobian as an index of a list: so whichever simplex we are in, query our Jacobian dictionary to use :D

REMEMBER:
- number of iters: RW application, 300 is large...... 30-100 is reasonable
- dont want to overshoot with robot - dampenign must be small

# May 30

Today I fxied a bug in the analytic Jacobian calculation which cleared up many question marks. I also added the pre-calculated Jacobian for each simplex (done so by either computing the central differences OR analytic JAcobian for the centroid of each simplex) and that method, when combined witht he simplex update (of coursE) works quite well! To compare, our method of updating the Jacobian using cnetral differences everytime we enter a new simplex gave us a total error of 60.597, while this new method of using the mesh jacobians (as we will refer to them now) gave us an erro ro f 9.1435.... And as our best possible model, the analytic Jacobian with an update at every single iteration gave us an error of 2.16902. All of these tests were done with maxiter=100 on our 2DOf robot.

Now we have some questions:
- Optimal number of simplices
- What range should we make the mesh over, especially considering that the robot movement joints can extend past that range?
- How can we use a camera with this?

Current problem is that we are oscillating between the same simplices, which defeats the purpose of designating counts of jacobian updates, since we can infer that the bheaviour will simply oscillate continually until MAXITER.

Something I wanted to do was assign certain regions number of Jacobian updates needed, because perhaps that would tell us important information about the nonlinearity of the function, but this oscillating is a problem. What if we try adding the simplices we have already entered into a list so that we avoid re-entering the same simplex? Will this only interfere with the convergence?

It would also be nice to present something for the lab during the Thursday sessions, but I don't know what yet. 

Currently implementing visual servoing feature. Should the mesh refinement be using the camera points or the real world points to create its mesh? In a real world application, we would like to make the mesh beforehand, and its certainly not practical to create the mesh with the actual robot, so I feel that the mesh should be made in simulation beforehand.



Mesh jacobians live in R3
but to be effrective for th ecamera, we still need the projected camera points :-(

do in sim just fkin, but in RW we know fkin so not useful

calibration phase beforehand for 5 min - once you have it you have it
^ if you do this you mgiht as well do camera claibration

focus on vs
^ fkin part of the problem

# June 3

The mesh seems to be fixed and working now, but there is a consistent symmetry that seems a bit concerning: surely it shouldn't be exactly symmetrical?

Maybe we could test it on a non-symmetrical known function, but for now let's focus on analyzing in some other way.

Perhaps we could look at the eigenvectors, or the singular Jacobian.

Notes on the Jacobian:
https://motion.cs.illinois.edu/RoboticSystems/InverseKinematics.html

- Error propagation: how much does perturbing one joint relate to error in the workspace?

> In general, in  nR planar robots, the  i'th column of the Jacobian is simply the vector from the  ith joint to the end effector, rotated by  90∘ CCW. ... if the end effector is farther from revolute joint i than joint j, then the magnitude of the i'th column is greater than the magnitude of the j'th column.

To employ IK:
> The robot's kinematic structure is known. The coordinates of the tool center point are known. The coordinate frame of the end effector link is known. A rotation matrix defining the desired orientation of the end effector link is known. All coordinates are given with respect to the assumed reference frame (and units) of the solver.

There are ways to navigate singularities and ways to identify them:

if we can make the mesh and generate some Jacobians and recognize that they are singular, we can strategically avoid them, right?


spectral radius is equivalent to the abs val first derivative in higher dims !!! DOUBLE CHECK!!!!!!!

Heith 5.6

start with newtons method and work in the analytic form - use a symbolic language 

look at the closed form jacobian, hessian 

- spectral radius
- singularities

hessian 

eigenvalues can be negative ==> eigenvalues are by convention always positive

# June 4

Today I made my spectral_radius.py program, to see the spectral radius of different Jacobians for different startign positions.

Found out two major things:
- The intial Jacobian that we start off with in the assigned-simplex-Jacobian method is always QUITE different than the analytic Jacobian, but it still lets us converge in most cases (though there is admittedly more oscillating)
- The spectral radius was quite low for many of the cases wehere we never ended up converging. So, what else could be causign our arm to diverge? 

TODO:
- Clarify the cases where low spectral radius and successful convergence can truly be correlated, since it seems there can be exceptions both ways.
- Run experiments over the entire space and see if convergence overall correlates with a lower spectral radius.

The spectral radius problem is mainly interesting in visual servoing: in regular inverse kinematics, the forward kinematics is known, but in visual servoing, the frward inematics must be approximated by central differences. 

I wish I had a clicky clacky keyboard.

TODO: Make a plot over the space ewith the spectral radius,

Figure out the nuances

# June 5

There must be something wrong with either my understanding of the spectral radius OR of my code, because I am getting spectral radius to be large 

# June 6

Today I am reading the Numerical MEthods in Scientific Computing textbook by Germund Dahlquist and Ake Bjork.

Things are looking up.


Global convergence ... Dottie Number

# June 9 

There are two methods we are focusing on right now:

Newton's Method: qn+1 = qn - B*F(qn)
and our current control method: qn+1 = qn - alpha * B*residual (where resisudal = curr-desired)

A lot of my confusions abotu the equations I was dealinfg wiht today were cleared up after reading the textbook and asking for help. Which is a good thing but I mourn the time spent being confused. I read a lot about polynomial interpolation and am currently reading about global search methods, which I can save in my back pocket for later. I am a little unsure what to focus on because I could look at:
- Broyden's update
- Global search methods
- Simply analyzing the spectral radius in higher DOF 
- Analyzing the spectral radius in visual servoing

I think it is more worth it to analyze the spectral radius first instead of moving to global search methods, since the objective of the latter is to improve convergence, but we will be better informed about convergence if we can make statements about the spectral radius.

TODO:
- broyden update vs true jac
- spectral radius analytic analysis
- consecutive broyden updates: does the err bn est jac and true jac diverge/converge/oscillate/etc
- analyzing without plots for higher dimensionality
- complexity vs scaling, cameras, etcccc, point to point VS point to line

two different ditection:
VS: scale thing sin terms of contrasints and cmaereas
- start one camera, try point to line, point to point, etc. start top down view, then add rotations.
OR in terms of JOINts

equation properties that allow scalability

Right now, my issue is the multiple roots: 'convergence' is defined regardless of the joint configuration, but analyzing each solution shows us a different map of spectral radii... 

Step 1: Look at the spectral radius analytic form and see what can be said of it. 
Step 2: Find some way to analyze the spectrtal radius of a system where we consider all of its roots... It makes sense that each root has its own unique plot, because we are simply evaluating if each desired position capably 'attracts' solutions... 
- min over both

ALSO how do we know which spectral radius is for which point? Like does each starting position choose its own point of attraction? 

# June 10

**cartesian space convergence - if form perfect circle**

*HOW TO DO THIS?*
- for every single joint angle pair we have, simply compute the forward kinematics for each point and plot it. Very simple! Don't overthink it!

relationship bn starting pos and goal? 
- translation invariance? add .2 radius to either, will it be same? 

if we trnalsate both the starting and goal by t SAMe amount, does the spectreal radius stay the same?

Big Picture:

**rn single jacobian: how can we add more jacobians? based on where the spectral radius is high?**

if we could do a couple central differences BEFOREHAND and still allow it to converge

if hessian flat, jacobian good
if hessian high curvature, jacobian is only valid for a small local region

can we use the condition number of a matrix

hessian not reasonable in practice

bisection in higher dimensions:
- in practice, that means in the robot we end up moving way back and forth - not good!!! very costly 
- bisection is also too slower... typically introduced early to get a good starting position, then switch to newton or faster method

More notes from Dylan:
If we have analytic invkin equation, then we can replace each u and v subs analytic form. Or, analytic fkin, with sym equations, collect 'collect' the fkin form  

It would be nice if collect: if equation is the equation for a circle

From Dylan's numerical experiemnts, seems to have a lot of circle shape going on 

Eigenvalues are conjugates of each other and when sin(v)==0, ie v=k*pi where k is any integer, we may expect no convergence. This makes sense, ... either we are folded on ourself or outstretched.
    
Can we make basins of attraction and compare that to our spectral radius plots?

TODO:
- basins of attraction
- cartresian space

Conversation with Tanner regarding the relationship between complex eigenvalues and spectral radius being < 1 in this case. 
Hmmmm..... Could it be coincidence, seeing that the forward kinematics equations are circular...? 
We have something interesting to explore...

if complex, ocillatory,
if real, non oscillating
root locus in control systems.

Let's regroup.

Currently it SEEMS there is a correlation between the spectral radius being less than 1, with complex eigenvalue, for the matrix A = del gn / del Qn = I - B @ (del F(Qn) / del Qn)

Can we designate certain points in the robot as jacobian update hotspots?

What are some questions we have?
- why why why is complex eigenvalue and spectral radius correlated here? It seems REALLY suspicious. how can we question that?
- by adding a non-circular function like e^x or x^3, I think we will defy this correlation.

# June 11 

Weights and Biases:
- website for ai simulations
- pass in dictionary/string thru their api to create a plot
- synchronize time steps between plots
- can upload videos and potentially matplotlib
- can log changes ... use functions from api 
- hyperparameters: human selected options that are not to be optimized, fixed for a run (ie learning rate)
- paramters - things that are updated, changed

Neovim 
- vimscript... archaic.... uses LUA instead ... easier to embed LUA in to different applications

Does damping cause the spectral radius of all initial joint configurations to pull towards 1?

How to talk to robot?
- Kinova is TCP but there is an API that hides the protocol details.
- Ex: API has move to joint pos, and that forms the TCP message and sends it 
- Protocol buffer which uses TCP 

Ros - big picture:
- message passing system
- different processes (nodes) allows you to publish topics: on the WAM or Kinova, there isa  topic where it is publishgint he joint states, and a different node can subrsricbe to that topic and read in mesages
- similar to buffer 

WAM:
- Tele-op uses UDP; we want low latency, send data, guarantees dont matter as much. 

Internal computer on Kinova - fixed, so no access to UDP. Little server running that opens a TCP port (maybe.....)
WAM has more flexibility 

Ros is used as a way instead of manually setting up sockets and by default TCP - topics, subscribe, publish

Ros inherently linux based .... problems!!!!

IP address for windows machine - wsl creates its own netowkr where linux lives - the problem with reos stuff is usualoly we need that newtowrk to be part of greater network 

VM might be easier - easier to bridge network. 

Dual booting: partition: part is windwows, part is linux. 
Linux doesnt take up a lot fo space, but windows does!!!!

Google "wsl2 bridge network with host" --> microsoft "accessing network applications with..." --> mirrored mode networking 

#### Okay Today, let's focus on our priorities.
- does damping cause the spectral radius of all initial joint configurations to 'pull' towards 1? ie bad points get better but good points get worse?
- what if the spectral radius can be associated to a specific number of jacobians... pre-compute jacobians... we do this because the mesh was too dense. But the problem is... is that the spectral radius is goal-specific. We want a goal-agnostic way of identifying nonlinear regions.
- let's shift some gears and write a visual servoing script. Maybe we could do it in C++, since I think just learning C++ would be epic

Plan for associating a number of Jacobians to the spectral radius:
- can we get something like joint spectral radius over the whole area and get the min? that seems really silly. what if we multiplied it over the whole area... could that be helpful? Can the analytic form tell us this right away?
- How would this translate into visual servoing? It's just another set of nonlinear equations, right? 

Spectral Radius VS Damping. My hypothesis is that all values will indeed move towards 1, since the limit as alpha approaches 0 in the equation I-alpha*B@dF  = I-0 = I, where the spectral radius will be 1.

After running a few experiments, it is true that as alpha gets closer to 0, the spectral radius asymptotically approaches 1, but for singular joint configurations, it appears that our spectral radius actually dips below 1 and then starts creeping back up to 1 again. 

# June 16

Over the weekend I dual booted my laptop. On Friday I spent a significant amoutn of time tyrignt o figure out Denavit Hartrenburg parameters.

I finished the code for the spectral radius for the visual servoing, but I need to analyze it further to be able to say anything worthwhile about it.

#### Comments from Dylan:
- Use two cameras: should have camera points > robot DOF, otw trivial
- avoid floats
- reassign bad jacobians with better ones

Today I am not sure what to work on next because I feel sort of stuck.
Dilemmas:
- We could replace singular jacobians with 'better' ones or jacobians simply to push us out of the singularities, but how can we ensure that our new location post push is convergent? Ie we can push ourselves into a new region and find a new solution trajectory, but it's not guaranteed that this solution will exist, and we really want to ensure that it can converge.
- Should I re-dualboot my laptop to Ubunut 20.04, or learn Docker to use 22.04.5? Because Docker appears to be a commonplace tool I thin it would be hlepful to learn, but I need to learn quickly so that I can still get work done. I can still use ROS 1 with Docker, which is good...
- Just running from simulation it appears that modifying camera parameters doesn't affect the spectral radius, but this feels wrong and it's highly possible my code is faulty.

Two objectives for today:
1. Figure out why the spectral radius is the same regardless of camera parameters (remember that extrinsic is more important to look at than intrisnic)
2. Set up docker and ros1 in ubuntu

Okay so we have a bit of a dilemma:

When we divide every entry of the projected image point by z, we end up with an array 
[x/z,y/z,1] aka [X,Y,1].
Then the jacobian's third row is going to be zeroes no matter the Q, so it will be not be invertible.

And when we use the psuedo inverse, we have spectral radius >=1 for points that were.... previously < 1...


I am having a lot of trouble finding points where the spectral radius is below 1 for the case where I project the point onto the x-y plane by dividing by z. This is the nonlinear aspect, since the z component becomes 1 and the inverse no longer exists. I dont need a ggraph, just a single point, so I should be able to easily make a huge iteration over the sets of pairs of joint angles. 

# June 17

I am very happy because I finished the simulation for the analytic visual servoing with two cameras using Dylan's 3DOF robot. When we only have 1 camera out of the two, we have a spectral radius of 1 or higher, but when we use both, the spectral radius goes below 1! I have only tested a few little points, so it would be good to make a linear space again. And it would be good to get the analytic form of the psectral radius, but I anticipate it will be quite lengthly...

Yesterday we set up Docker and ROS a little bit and I would like to get started on the actual simulations.

#### Lab Meeting:

Look into floating planes and read A Mathematical Introduction to Robotic Manipulation

Big picture:
- create a real time (online ?) method that, given an ill conditioned jacobian, will replace the ill conditioned jacobian with a better one
- beforehand, compute the central differences for n number of jacobians, and for ANY trajectory or goal we want, we will use max n jacobians

We can sit on these ideas and shift gears to real experiments.

# June 25

Past week and a bit had been focusing on Ros and Docker, etc, but it has taken a bit of time and I want to focus back on the mathematical analysis part....

Let's assume we HAVE the spectral radius everywhere...

Then if we indicate we want to take on a certain trajectory, can we... get the jacobian of the final destination and 

bayesian opt: query around datapoints, select next datapoint - gaussian processes

# June 27

Finished setting up the visual servoing on the Kinova which I am very happy about!

Now we REALLY have to focus on the MATH. 

# June 30

Nice to haves:
- don't need analytic calculations (I mean, we CANNOT have it since we are projecting it down ANYWAY)
- 

When we have a goal position for the arm, how can we generate an intermediate pose for the arm to go to so that we can seperate the movements?

We have 

TODO:
- try using linear combinations of n predetermined Jacobians. and maybe this will also give us a sense of depth. It is admittedly very difficult to discern the depth, isn't it...


This is a kind of random idea but there IS a pretty strong correlation between the cartesian position being NEAR the goal position... And this also has to do with the fact that, if all joints are in the correct position except the furthest joint from the base ie the hand, it seems easier to converge. (Is this true? We should try this out.)

It feels like just looking at the cartesian radius of a joint is too simple, but we don't have access to that much infromation anyway, and during visual servoing, we are allowed to look at the projection. We can draw circles to gauge the radius on the joints in OpenCV or something. 

Maybe we could draw a circle around the joint, gauge if we think one jacobian will do it, and if the goal isn't near the perimeter of the radius of the circle, that indicates we need a little more action... move the base arm until the radius of the joint touches the goal.

This feels very naive, but maybe we could just try it anyway.

If we go along with this idea of reconstructing poses, it becomes very problematic if two cameras reconstruct two different poses. Let's ignore the camera conundrum for now.

What if instead of computing ellipses, we see: for each joint, which camera sees a greater amount of perturbation? Then, generate a circle in that camera: so each camera has one 'circle' for each joint.... and for each step we can reevaluate circles

This entire time i have been thinking of the problem very much as a system of nonlinear equations, but when we think about the fact that a local region in cartesian space often converges and that we have access to OpenCV and cameras in visual servoing, I realize we have many more tools than we think.

Of course this "draw a circle and go there" is not very reliable, especailly if:
- perspective looking at circle is skewed
- we have a camera on the robot

nevertheless, this could serve a good TOOL

# July 2

Conversation with Dylan and Cole:

Creating trajectory intermediate steps and associating that number of steps with number of jacobian updates: can we take a trajectory to get to a goal position and check every step, see the area of convergence to get to that next goal position (see the farthest point that it can successfully converge to, or work BACKWARDS: from the goal position, find the area around that circle that can converge to the goal)

So, instead of my previous idea of focusing on each joint and 'toggling' the joint to get closer, try a more general approach of looking at the Jacobians.

When we look at the planar 2DOF arm, we can see the x-y convergence areas clearly, but I wonder if it'll still work for 3D cartesian space. But we are just trying to see what works and see the actual minimum number of jacobians here, so just use the analytic jacobians.

Hypothesis: the closer we are to a singular matrix, the smaller the convergence region to that point is around that point. 

Result: yes it's true! the farther we are from the singular outstretched position, the larger the convergence radius is. When the desired position is ORIGIN, we have a huge region of convergence in cartesian space, but this could be attributeed by the multiple solutions that exist for that orientation. It's hard to say if this is a valid observation yet.

So, workflow right now (and let's do everything analytically, since it's best if we can make some observations and then apply)
- for an ets, see where the singular jacobians are, and make note of them!
- then get the region of convergence for a constant jacobian around that area. (and note that damping can change all of this, so let's try without damping for noe, since we can see how damping changes things LATER)
- test if our 'theory' works by generating a trajectory of jacobians.... ;-;

- what if we store all the success/fail points for each point that converge to the GOAL POSITION --> then from the set of success points, we choose the closest point to the CURRENT GOAL POSITION and make that our intermediate step. REPEAT until we get to the GOAL.

# July 3

Consider goal to init VS init to goal (pros of the latter: you could KNow the joint angles)

- practicaltiyh
- smooth the trajectory steps? --> even if jagged looking cartesian solution, the joint solution might be smooth :0 ...... might end up get something like that anyway ewhen you end up with a full solution bc the approx of a rad of convergence is CONSERVATIVE, so likely more steps than needed. --> might give u more smoothing 

# July 4

So far we have a simulation that works backwards from the goal to find all points that converge to the goal, identify a successful point in that region that is closest to the starting end effector position, and make that successful point a milestone in the cartesian trajectory. 

There are two topics (aka potential issues) we need to address right nwo:
1. TRUE NUMBER OF JACOBIANS: We can successfully compute a number of jacobians, and the number is often low (2-4) which is a very good sign because we should never really need too many especially for these smaller problems. HOWEVER, there are times when 1 Jacobian is necessary but our algorithm calculates 2 are needed or so... Part of the problem is we have to specify a tolerance to qualify the number of jacobians as appropriate, and depending on how we geenrate the mesh, the tolerance needs to be meddled wth, and of course this is not genralizable. fix this we had to put in bandaid solutions of a sort: when we sample from the same points each time, we can get 'stuck' trying to refine further when there really are no more 'closest points' and the algorithm jsut comptues the same point over and over again. So the solution to fix that is to cache that closest success distance, and then check on the next iterartion if the distance is the exact same. OR (and this is actually a much better apporach) to randomly sample points all over the space. We could keep the same points or get new ones, perhaps it doesn't matter and perhaps one has an advantage. 
2. There is also the issue of moving out of bounds when we have multiple milestones.

Time to read some papers on...
 - trajectory generation

4-D point cloud  --> exact same process of which overlapping points --> account for discrepancy between multiple solutions 
 - how to decide which joint is most 'effective'

this raises an interesting question at looking at the joint space and the cartesian space together.
how can we do this with the real robot though? Still no idea so we might as well gather a lot of insight in simulation.
Earlier I had chosen to get the region of convergence in joint space because the multiple solutions cause some ambiguity, but if we are simply choosing the nearest point each time, we would be travelling towards the correct joint space solution --> thinking about the basins of attraction --> how can we pay attention to which basin of attraction the robot wants to go in? This seems like a possible area to explore.       

Tanner has suggested creating a 4D space with the cartesian x,y,z, and the base joint. This sounds like a good idea. 

where to research
- fkin --> GIVEN jcaobian, use to solve and plan optimal path
- kinemaitcs under uncertainty --> ML

keywords:
'uncertain' 'unknown' kinematics path planning

could look at calibrated form --> is calibrated different than uncalibrated 

globally approximate the jacobian with ~100 training data samples 

new directions --> less samples OR real time (avoid waving around in unfamiliar environment)

you CAN learn it globally online, but its not useful if you have a different target where you want it go --> ok keep updating over time everytime see new target, broyden update. but if we are recomputing everytime we go somewhere new, we defeat the purpose of learning the global function. 

What if we comput eall the poibts that the initial configuration can converge to
ANd all the points that converge to the goal

Imagine multiple permissible and possible milestones. Our goal is one of them. 
If we could look at the basins of attraction for the goal, and the closest point in the goal, and only 

Get the basins of attraction for the GOAL
Find the shortest distance between the intial joint configuration and a joint configuration in the identified basin of attraction. (this is essentially repeating the closest success point again but in the jonit space.)

# July 7

Refocus: what is the objective? To quantify how many jacobians are needed to converge to a certain point in visual servoing. 
We do not have access to analytic equations. We will not have a calibrated system. 
Can we use the image projection of the robot in some way? 

Objective: 
- is there an equation to determine how many jacobians are needed for a desired and an initial point/configuration? Can we determine that based on how large the spectral radius is? Can we generally eyeball it? How would our assumptions hold in a robot with prismatic joints?

Some things we have noticed:
- The joints do not like to flip. For instance if you have a 2DOF robot with joints -0.1 and -3pi/4, instead of swinging the entire second arm by pi, the first joint moves up by pi/2 while the second joint only has to open up by around pi/4.

REVISIT the basins of attraction: Perhaps this was obvious, but the basin that a joint configuration is in is typically the solution that minimizes the simple difference between that joint configuration Q and the solution Q. This indicates that idea we had before to simply move the 'most effective' joint is probably not the best idea, since neither arm can quantitatively more important. For instance in this example:

initQ = np.array([-0.1,-3*np.pi/4])
desiredP= np.array([1.0,1.0,0])

There are three milestones: Beginning, Middle, End: [ 0.17474149 -2.46174514], [ 0.88614019 -2.07347846], [ 1.57144681 -1.57220103]
This translates to 2 jacobian updates needed...

# July 8

Trajectory generation in joint space vs cartesian space:

### Jotnotes from Chpater 13 Trajectory Generation from Science Direct:

https://www.sciencedirect.com/topics/computer-science/trajectory-generation

Trajectory generation in the joint space is desirable to achieve FAST MOTIONS in a FREE SPACE

PROS:
- requires fewer online computations, since there is no need to compute the inverse geometric or kin models 
- trajectory is not affected by crossing singular configurations
- max velocities and torques determiend from actuator data sheets.

CONS:
- corresponding end effector path in task space is unpredictable
- risk of undesriable collisions

Trajectory generation in the task space permits prediction of geometry of the path, but:
- may fail when the computed trajectory crosses a singular configuration
- fails when the generated points are out of the joint limits or when robot forced to change its current aspect
- robot is slower

The trajectories of a robot can be classified as follows:
- trajectory between two points with free path bn them
- trajectory between two points via a sequence of desired intermediate points (aka VIA POINTS) with free paths bn via points
- trajectory bn two points with constrained path bn the points
- trajectory bn two points via intermediate points with constrained paths bn the via points.

*Perhaps this means I should focus on visual servoing trajectory generation...*

Cole:
- joint space traj gen is easier
- task space has different ways to do it: ie takign intermediate points and inv kin 
- visual servoing --> planning in the image space. probably not taking joint space 

### jotnotes from craig robotics chapter 7 on trajectory generation:
- it is generally easier to find trajectories in joint space so that singularities are not an issue
- something interesting is, for linear parabolic blend splines, if you choose a via point and then interpolate the trajectory around those via points, you will not actually pass through those via points. If you want to go through them, you should plot two via points next to each other and the algorithm will naturally force the interpolation to pass through the LINEAR path between them. (if we did this for a different algorithm, we might have the risk of something like Runge's Phenomenon perhaps, where there is a great deal of spiky behaviour in between two very close-together-points.)
- collision free path planning seems very interesting:
> systems that plan collision-free paths are not avaialble commercially... One approach solves the problem by forming a connected-graph representation fo the free space and thren searchign the graph for a colision-free path. CONS: exponential complecxity in the number of joints in the device ... The second approach is based on creating artifical potential fields around obstales, which cause the manipultator(s) to avoid the obstacles while they are drawn toward an artifical atrective pole at the goal point. CONs: local vieew of env, subejct to becoming stuck at local minima of the artificial field.

### jotnotes from "Online task space trajectory generation" Daniel Sidobre and Wuwei He

> Trajectories are time functions defined in geometrical spaces (Cartesian or joint space)

For trajectory planning, the lower degree the polynomial is, the better.

*What information do trajectory planning problems also have/don't have?*

TODO:
- backwards and forwards concurrently? find where they overlap
- forwards --> more feasible trajectory?

### jotnotes from optimal path plannig for image based visual servoing Mark Allen

> key factors in handling non optimal state trajectories and factors on how to set up a dynamic programming algorithm for IBVS were discovered: the first is that the image Jacobian matrix must be a square matrix, in order to provide a deteminant that gives information on the local behaviour of the image function including mapping. Due to the image Jacobian for one point having dimensinos of 2x6, exactly 3 points are required for the dynamic programming approach.

- calcualte backwards, starting from known final position. (which we also have!)

## takeaways:

after reading some papers/resources, what do we think?
- trajectory generation is usually completed in the joint space because the task space leads us into singularities more often.
- we will have access to the joints during real world visual servoing since we initialize to a specific pose and move from there. 

What if we sampled the jacobian at a bunch of joint space points? Then we have:
- the joint space vector, central differences jacobian for several unique joint space positions, and the corresponding task space vector. 

using those jacobians, we can see how different they vary/the landscape of how nonlinear they are. This helps tell us where/how we should update the jacobian. 

For instance, if we are given an initial pose and a desired pose, do the following:

For the given desired task point, can we, using the precomputed points, 'guess' the corresponding joint vector, compare the initial vs desired JOINT vectors, and then guess how many jacobian updates we will need based on the joint space? But this is difficult because of the multiple solution problem.

But this seems like a flawed approach... because it's really not generalizable. So, we know that there exists one task space region where only one jacobian is needed. And it's helpful that it is just ONE region. 



Choosing our error vectors:

if successful convergence from initial to milestone,
iP - desP must be small to indicate this is a good milestone.
currmilestoneQ (aka initQ for now) - iQ must be smaller OR currmilestoneQ-Q must be smaller so that we can reasonably get there.

trajectory generation is NOT an inverse kinematics problem

chaptter 10 craig robotics
 - trajectory conversion, you KNOW the jacobian for ANY points basically. we KNOW the inverse kinematics 

 try existing visual servoing traj gen experiments

 try on the robot:
 - use a const jacobian, send the goal, try to get as clsoe as possible, monitor how the eror is changing, if error not decreasing fast enough, compute a new jacobian.

 from singular point to singular point, we definitely need MORE jacobians.

Currently focused on trajectory generation, but need to still consider nonlinearity and complexity of the landscape as a whole...

Perhaps we should focus back on the analytic equations and the math. 

TODO
1. Is there a measurement similar to the spectral radius to see whether a point may converge? 
2. What are some properties of nonconvergence? --> singularities, boundaries
3. What is the NON-singularity reason why some joint configurations do not converge despite NOT being in a singular position? Well, the spectral radius is usually around 2, and the joint trajectory usually oscillates in a LINE --> the eigenvalues are real, while the converging eigenvalues are imaginary.
4. Side note: we are able to compute the perfect amount of damping at any moment

For 2DOF, remember the spectral radius equation? See if you can find a relationship in there: if v is very low, then we get stuck at a singularity. But there should be a relationship between u des, v des, u, v, explaining convergence. Well... remember the imaginary vs non imaginary? Just look under the square root.

But what about being able to converge in general? What about creatign trajectories? Very difficult...


# July 9 

If we can guess the radius of convergence, we can guess how many updates will be needed... since we can just overlap them...

### Pre-calibration: sample over the space, associate joint space to task space. 

Then when we have a goal, we can search the BEST PATH to get to goal... 

We know that the MORE SOLUTIONS EXIST, the larger the region of convergence... So can we use something like CHEBYSHEV sampling to sample more on the workspace boundaries.... And we know that the basin of attraction well suited to us is the one which the joint vector is CLOSER so we can add that in the evaluation step...

Is my objective to create a method of pre-calibrating, or to explore the mathematical properties?

generate a bunhc of joint configurations.. get the end effector...
for desired point, get the multiple  corresponding solutions and compare which solution is closer to the current joint configuration... then that is the solution we should try to aim for...
what is the relationship between the radius of the convergence region and the desired position and the initial position? We could find this out using the spectral radius but its so difficult to even obtain.

Focus on upper/lower bounds?

Check the difference between the current joint configuration and the solutions: maybe there is some relationship there...

Do i want to find something THEORETICAL or something PRACTICAL?

T_T

TODO:
- Bounds
- Simple frobenius norm diff in curr joint -  the des joint solution
- kshbdfskhdbfkshdbf

# July 10 

Let's find some way to get the bounds of:
- the radius of convergence in task space
- the radius of convergence in joint space --> multiple solutions so this is potentially harder.
- the radius of convergence in task space given the joint configurations 

is there a relationship between the nonlinearity of the jacobian and the radius of convergence? Well, it would be surprising if not.

Over the space if we can see how great the change in central differences is --> or just get the condition number of the matrix, since if there is great change or nonlinearity present in the matrix, the conditiion number will simply be a bit higher at that points: and in that case we can really just get the norms or the norm bounds.

The thing is, the analytic bounds get really complicated, really fast.

What is something analytic we can look at the discern the radius of convergence?

I found a paper that uses the osculatory circle to approximate the small epsilon-like amount to add to an initial guess at singularity or local extrema so that you can push yourself out of the ill conditioned region and back to finding the solution. 

Lipschitz function: relates to the LIMIT of the rate of change of a function. Uses the double cone (looks like an hourglass) along the function and the function must never overlap the cone area. The smallest shape of the cone is the lipschitz constant.

If there exists this method to get out of singularities for single variable fn, is there multivar? Very very likely, so let's take a look.

Use the lower DOF case to justify approximate methods for higher DOF:
So, the more singular the condition number of the goal position, the smaller the regions of convergence and the more jacobians we will need to converge. Also it seems the higher DOF we are, the mroe jacobians we will need.

But just getting out of the singular position is tough and it helps to perturb a little bit initially to get out of there.

Is there an analytic way we can show:
Given an initial point and a desired point, how many jacobians and where should these jacobians be sampled? 

We also need to highlight the fact that damping should be adaptive becauase of the way that the spectral radius goes to 1 as damping goes to infinity, that's pretty rad! (Maybe revisit, why does it actually, and what does it mean, that the spectral radius actualyl dips below 1 jsut a little bit?)

So if the spectral radius is 2, where should we redirect ourselves? There are multiple paths to go from here because there are multiple reasons.
- REASON A: The spectral radius is 2 because we are actually well positioned to converge to the task space but with just a different joint solution. But we would only know this if we had access to all the solutions prior. 
- REASON B: The spectral radius is 2 because we are near a singularity, though.... the spectral radius of the singular points are often extremely high and the spectral radius gets small again very quickly.

Remember when we looked at the spectral radius plots for 2DOF arm? They had a clear boundary in them where the second joint == 0 (the singulartiy where the joint was fully outstretched) and it's like.. walking across that divide is very difficult... And if we aren't sure where the solution is, it's difficult to decide if we should cross that boundary, or move closer to our nearby solution. 

Isn't the root that the solution will choose simply the root such that minimum(norm(ith_root-current_joint_vector)) out of all roots?

So this is some way to predict which root we will CHOOSE, but this is more of an empirical thing, and we need to focus on analytically, how can we handle the multiple soltuions? It is worht tryign to figure out how to converge to a very hyper specific solution even if there exists a better solution at hand? Well, if we can navigate singularities, then we can try to make every single possible point converge to that specific solution, but then we have this issue of:
- If we know the solution, then we know the joint configuration, then we don't need inverse kinematics.

So go back to looking at the jacobian... if the region of convergence 

OK so what we need to rely on is....
Can the condition number of the jacobian of a certain joint configuration (since this is goal agnostic and only needs the current joints) tell us its region of convergence? Well, we've seen that yes it can tell us quite a bit, since lower == better.

Then after we obtain the guess for the region radius shape etc etc,,,,, how can we guess the number of jacobians it will take for us to converge? 

And another thing, how can we include damping in these calculations???

Fully rely on the jacobian to tell us about the landscape and how nonlinear the landscape is in cartesian or joint space? 

PROS of joint space:
- smoother, avoid singularities
- multiple solutions, how to navigate?
- much bigger than cartesian space, but way more informative
- but it is TRUE that the singularities are like barriers, so the question is, how important is it that we can step over those barriers? Well, right now we are kind of saying it's not important, but that is only because we assume there exisst another solution on "our side of the barrier", but what if we believe the grass is greener on the other side for real and there isnt a solution near us? Then in that case it is honestly pretty imperative that we can step over the singularity barrier, but only after checking for certain that there isn't a solution nearby that we should just move to instead.

PROS of task space:
- the convergence region is typically one region and not multiple confusing basins and stuff

Wait what if we just looked at both spaces at once? 

We also need to decide if we should traverse online, or compute evrything beforehand.

We kind of need TWO steps:
- ADDRESS SINGUALRITY: are we in a singular position? If so, we should get somewhere better. This is not a difficult problem. The main qualm is just: which way should we perturb?
- ADDRESS WHICH SOLUTION: which solution do we belong to, or can get into? This is far more difficult, because it is goal dependent. It is starting to feel like we will need to sample over the space and interpolate positions, etc. But that brings us back to Cole's point: if you can access the joint positions that are near a corresponding cartesian point, then why NOT just 'go there'? Then you don't even need inverse kinematics. Maybe we are thinking about this the wrong way. 

What if we only look at task space? For each joint configuration, get the radius of convergence for that specific joint config... 
and joint space trajectory planning is uncommon anyway so that could be soemthign interesting to explore anyway.

If we sample over the joint space (and thus the task space), we might as well just interpolate the solution and GO THERE.
Let's try something in real time.

Look at the condition number and GUESS where the radius in JOINT space is. So, given one initial point, how many points can it converge to? 

So what is the relationship between the condition number and the region of convergence?

Question: as long as youre not in a singular ill conditioned position, is it possible to converge from all points eventually? Right now we are insistent on providing a way for the robot to find its natural basin of attraction, but why don't we provide it simply 

How does condition number relate to the radius of convergence?

TODO:
- Make a simulation that gets condition number and compares it to the convex hull of the convergence region- the largest possible radius? Well, it's not a circle, so that complicates things, but maybe it will be OK. 
- If jacobian is singular, then the joint that is responsible for the singularity should be perturbed (and we can identify the joint) and whichever direction decreases the loss, that is where we should move.
- Move by how much? Ans: something like the osculatory radius ....
- Okay we have moved. Now re-evaluate the condition number and go to the point that is magically closer.

During this process, should we update and store/cache information as we go, or just have a milestone and go there and then re-evaluate?

For now let's focus on just getting the condition number, and the convex hull of the convergence region.

# July 12

If we want to do this as an online algorithm, we need to make some sacrifices:
- We cannot guarantee convergence for any arbitrary initial and goal position: we can only say we reduce the number of jacobians updates needed by approximating the region of convergence. REASONING: imagine we are in a singular position. Our first priority is to get out of that singular position. To deal with the singular position, maybe we can: perturb the joint that is most responsible for the singularity, thus providing two possible solutions... and whichever solution has less error we can use...

Two things to focus on:
1. Radius of convergence (how many iterations should I travel given the jacobian? Can I guess the radius of convergence of Newton's Method?)
2. Getting out of a singularity

# July 14

Kantorovich Theorem

Kantovorich Theorem can give us results on semilocal convergence.

There are FOUR CONDITIONS on operator F and starting point x0:
Let X, Y, be Banach spaces, C contained in X, F: C-->Y a continuous function cont differentiable on int(C). 

K1: for x0 in X, there exists inv([F'(x0)]) in the set of bounded linear operators L(Y,X) st the operator norm of this inverse jacobian is <= some constant BETA
K2: the operator norm of the inverse jacobian of F at (x0) @ F(x0) is <= some constant ETA
K3: There exists a constant Lipschitz >=0 st the operator norm of [F'(x)-F'(y)] is <= L*[norm of x-y] for any x,y, in the subset C in the Banach space X
K4: h = Lipschitz * BETA * ETA <= 1/2 and B(x0,R) st R = ETA * [2*(1-a0)]/[2-3a0], where if a0 is >0 but <1/2, then the Newton sequence has R-order of convergence of at least two, while if a0==1/2, the seuqence has at least 1-order of convergence,

If K4 is valid, then we have guarenteed that:
- a solution exists
- a solution is unique
- we know some bound on the rate of convergence to this solution

The first two conditions are on starting point x0.
The third is on operator F.
The fourth combines the first three.

So, some takeaways:
K1 tells us that the inverse Jacobian of the starting position must exist (aka J is nonsingular). So, if we are initialized (or wind up in) a singular position, the first priority is to get out of that singular position.
K2 tells us how near we are to a solution already: this is, after all, the size of the first step! And remember that F(x0) is the error function that should == 0. The higher b is, the farther we are from the solution.
K3 tells us we need Lipschitz continuity of the Jacobian: the Jacobian change must be bounded by some LINEAR constant. The higher L is, the more unpredictable and nonlinear our function likely is. This tells us how predictable the region is. 
    Papers that focus on the domains of parameters and increasing that region focus on tuning the Lipschitz constant, which is something we should focus on.
K4 is THE condition and is called the Kantorovich condition! It simply tells us whether or not we have guaranteed convergence.

Here is some information we need to gather:

- Error function that maps DOF variables into a 3D task space (However, we will have to work in the 2D task space eventually): F(t1,t2,...tDOF): R^DOF -> R^3
    We can easily grab this from the Denavit Hartenberg Parameters Matrix third column (the translation vector)
    Needs the forward kinematics/ DH parameters. EASY
- The global lipschitz constant L.
    This may be quite difficult to obtain over a large joint space. This is a bit of a problem in and of itself. L = sup((operator norm of F'(x)-F'(y))/(operator norm of (x-y))
    It is potentially a little silly that we scrapped the other 'compute/calibrate beforehand' ideas but must go ahead with this, so we should see if there is a way to predict (or if there exists a general lower bound we can find) for this Lipschitz constant.
    Hmmmm... I also wonder if it is possible to get the Lipschitz constant locally, if that'll do?
    Needs the Jacobian of the error function. HARD
- The constant b, st the operator norm of [F'(x0)^(-1) @ F(x0)] is <= b. 
    This should be easy to attain, since we simply need to: compute the central differences, invert, and mat mult by the curr error fn. b= the constant obtained from this Newton step calculation :-)
    Needs x0 and the Error Function F. EASY
    
So, we can gather that, since we need the GLOBAL lipschitz constant, that this is a very conservative, safe lower bound. 
This tells us though, that for function landscapes with very high Lipschitz values, the radius of convergence may end up being far too conservative!

So, there is certainly a lot of value in deciding not to even go NEAR singular positions, since it will likely enlargen the radius of convergence. HYPOTHESIS 1

We should get started with coding the first three values:
- Error Function F
- Lipschitz constant L
- Initial error constant b

1. Find the Lipschitz constant
2. Are the first 3 K conditions upheld?
3. Find a way to make the 4th condition true, and then calculate the n-d ball of convergence.

It is possible that this ball of convergence may be very small for larger and larger DOF. 

After we explore Kantorovich, we can look into the parameters of convergence.

# July 15

compare local vs global lipschitz constant of jacobian over the 2d space 

Today we are dealing with the problem of approximating/reasonably obtaining the global lipschitz constant.

The global lipschitz constant of A is equal to the spectral norm of A == the supremum of the absolute value of the largest singular value of A == the squareroot of the largest eigenvalue of ATA == the supremum spectral norm of A.


### On Using Estimates of Lipschitz Constants in Global Optimization Hansen et al 

> No finite algorithm can guarentee obtaining a solution within epsilon of the optimal one, for any positive epsilon. Indeed, f might contain a very high and narrow peak which might be missed by all the function evaluations even if they are very numerous.

Use sawtooth iteration and continually build the sawtooth cover to find its maximum. Kind of like refining the mesh.
L = max slope found, then multiply max slope by some factor r>1, (recommended r=2) since the slope is potentially very very large...

Start with global end with local seems to be a recurring theme...

### Lipschitzian Optimization wthout the Lipschitz Constant

DIRECT algorithm seems very appealing based on conclusion: has only one parameter that appears to be 'faily insensitive' and eliminates need to Lipschitz constant by carrying out simultaneous searches with all possible constants. Can also run in very high dimensional spaces. Does not require derivatives.

# July 16

Still looking for a global lipschitz constant solution.

Worst case scenario if we cannot find anything: sample over the joint space and multiply that by a factor > 1, since it is highly likely we will always underestimate the true Lipschitz constant from purely empirical guessing, and it is better to overestimate the constant anyways.

### Enlarging the domain of starting points for Newton’s method under center conditions on the first Fréchet-derivative, Esquerro, Hernandez-Veron

This paper suggests we can find better starting positions for Newtons Method based on a starting position we already have, even if that initial starting position is not necesarily very good. 

Many papers also use the local Lipschitz constant to enlargen the domain of parameters, but this is quite restrictive. 

Some authors require that, instead of Lipschitz continuity, we must have Holder continuity: 

There exist two constants K >= 0 and p in [0,1] such that operator norm(F'(x)-F'(y)) <= K* operator norm (x-y) ^p

Now we are using instead of a Lipschitz constant, a function in the usual Lipschitz inequality. This function w is a nondecreasing function.
 
*What is meant by 'center lipschitz'?*

Condition B2b: 
There exist x~ in X and two continuous and nondecreasing functions w~ : R positive --> R positive and h~ : [0,1] --> R positive such that w~(0) >= 0 , w~ (tz) <= h~(t)w~(z), with t in [0,1], z in [0, inf), and operator norm of F'(x) - F'(x~) <= w~(operator norm of x-x~), x in X

What does that even mean? 

Region of Accessibility: The set of starting points such that an iterative method will allow any solution of an equation from any point of the set to converge to the solution.

### Newton's Method: an Updated Approach of Kantorovich's Theory

There exists semilocal, local, and global methods of studying convergence.

SEMILOCAL: demand we know initial x, find domain of parameters to get ball of convergence
LOCAL: demand we know final x, find domain of params to get the region of accessibility
GLOBAL: convergence of sequence xn to the solution x* in a domain and independently of the initial approximation x0.

Essentially, is there some Newton-Kantorovich method that mitigates need for the Lipschitz global constant?

OPTIONS according to the World Wide Web:
- Center Lipschitz
- Lipschitz + Center Lipschitz
- Holder continuity
- Weak Lipschitz

Allegedly there is a way to, given a starting point, find a better starting point, but it's very difficult to parse the paper and find the actual method. Right now we are scrolling through the same proofs again and again...

### Newton's Method under Different Lipschitz Conditions Jose M Guiterrez and Miguel A Hernandez

The 'center Lipschitz' refers to the Lipschitz condition with the fixed x0. 

The center Lipschitz is weaker because one of the points is fixed.

There is another Lipschitz condition on the first derivative (thus requiring the second derivative) that is more restrictive.

Okay I cannot really find too much more.
For now we can try:

EMPIRICAL LIPSCHITZ:
Sample over the space to get the lipschitz and multiply by 2.

To get the analytic Lipschitz:
- Get multivariable operator F
- Get the Jacobian J = F'
- Optimize over the joint space to find the maximum spectral norm in the given bounds for each variable.

To get the analytic Lipschitz, we are currently lambdifify the sympy Jacobian into numpy/scipy expression, evaluate with a guess, and then optimize using scipy minimizer (minimize the negative function to get the maximum).

Approximate the Lipschitz... we should do random sampling or halton or sobol sampling. Uniform grid sampling is really expensive.

Today we figured out what center Lipschitz is. 

Tomorrow let's focus on getting more code down.

# July 17

Today we compare the optimization minimzastion strategy VS the random sampling fo 50 points to get the Lipschitz Constant. We need to be aware, though, that if we sample too much, it defeats the purpose of our strategy since we might as well solve the problem using interpolation or something.

Right now the issue with the global lipschitz constant is that we don't want to sample over the entire space. 

TODO: 

- test difference between global lipschitz constant VS local*2
- camera projections
- check how damping affects 
- does pinv mess up calculations? 

# July 18

Algorithm Flow:

Calculate for a specific initQ and desP: norm of inv jac at initQ, spectral norm of the jacobian at that spot, and norm of the first newton step.

if h is <= 1/2, we should be able to simply converge using a constant jacobian..

ELSE

inv jac at initQ. If jacobian is infinite, we are in a singular position --> identify the culprit joint, and perturb just that joint at forward, backward. obtain the kantorovich variables for forward, backward, and choose the solution with the lower h. 


If the spectral norm (norm 2) of the inverse (or pinv) jacobian is infinite (or just very high), then we are in a singular position and should just get OUT of that singular position.

 and determine whether or not we can converge (h<=1/2)

If we cannot converge, then we need to determine WHY.

Okay either we are in a singular position, or our first Newton step wants to explode over because it's just no good. 

So if we are at a singualr position, we've already come up with a way to perturb out.

But if the newton step is too big, then can we DAMPEN the oeprator F (aka the error function) and that's basically saying "can we converge if our goal is a little... just a little closer?"

And since we aren't singular there, I feel like it should be okay.

Are we necessarily avoiding singular positions? Can we say, given that we don't START from a singular position, that if we are caught in a singular position, identify the troublesome joint and perturb it in the same direction that got us there to begin with?

IF we are too far from the desired position and our Newton step is very alrge,  we do something like a logarithmic search and dampen the operator by halves until h <1/2 and then add to the dampening until h == 1/2 or so, and then make that our MILESTONE?

For dylan's 3DOF arm, the lipschitz constant is around 0.9.
For desP = [0.3,0.,0.0], the initQ is very weak and h is only < 1/2 for values EXTREMELY (within like 0.1) of the actual solution. This is no good, becuase the radius should not be so weak.

When we modify the newton step condition and add the alpha damping amount, it (obviously) decreases h, but I'm not sure if I'm allowed to just do that...?

### H.T. Kung: THE COMPLEXITY OF OBTAINING STARTING POINTS FOR SOLVING OPERATOR EQUATIONS BY NEWTON'S METHOD, 1975

This is a very interesting paper that does away with the whole 'assume we have access to good starting points' and instead says, okay if we have a bad starting point (and we can discern if it is a bad starting point by employing Kantorovich's method) then we are actually equipped with the tools to find a better starting point which is AMAZING and maybe exactly precisely what we have been looking for because it also does not seem super expensive, since we only need to make one newton step every time, and I don't believe we even need to obtain the jacobian ( need to look a little closer but it seems we just need to get the norm of the operator evaluated at that point, which is epic. )

Workflow of things to implement:

1. Kantovorich conditions
2. (Try with and without this step: If SINGULAR, then move to a nonsingular position)
3. If not met, then use the HT Kung algorithm to find a better starting position
4. Figure out where to use damping

And after all of this, compare using the global Lipschitz VS the local lipschitz * some number (if there is a relationship between the lipschitz and the singularity of the matrix, then we can have a smarter way of figuring this out.)

And we should add dynamic dampening

So essentially what we have is a program that uses the least amount of Jacobians possible, since we can find a better starting point and then use that constant jacobian to converge.

This isn't exactly what we WANTED to find out before though... we wanted to discern how many jacobian updates were needed from one region to the next... maybe that is too difficult to complete online-- because if we pre-computed many jacobians and stored them offline for online use, then... we might as well look at those JOINT positions near the task solution and say Well why don't we just GO THERE instead of solving for ANYTHING at all.

Hopefully this is going somewhere interesting.

# July 19 

Harrowing news. The HT Kung method requires we know the function f so that we can generate homotopic functions.

Before, we could sample the function and explore the function heuristically. Homotopy has different needs.

# July 21

So, if we want to solve an easier function than F, we could choose a few points and interpolate them to try to make some spline. But what points do we need to even choose?? 

Reading on Homotopy Methods in the Handbook of Global Optimization:

So right now we have a method of checking whether or not convergence will occur.

If convergence fails, what can we do? --> Find better starting points.
But to find better starting points, we need to use homotopy methods. YET F is unknown. !!!

## Handbook of Global Optimization W Forster

### Lipschitz Optimization Hansen, Jaumard

It is easier to solve global optimization if we make the assumption that the slopes are bounded: thus the functions are LIPSCHITZ.

Univariate lipschitz is studied most.

Multivariate: more complicated since it is hard to define a 'best possible algorithm' to be used as a basis for comparisons.

Constrained lipschitz optimization has been much less studied than unconstrained.
*Section 4* covers CONSTRAINED 

Problem 1-3:
maximize f(x) (1)
subject to:
    gi(x) <= 0 , i=1,2,3..,m (2) (where g is the constaint, ie x^2 + y^2 - 1 <= 0 constrains the point to be inside the unit circle)
    x in X (3)

minimal assumption to allow solution of problem 1-3 is that the functions f and g ( the operator and the constraint) can be evaluated at all points of X. 

Problem P: find a solution
Problem Q: find the set of global solutions

Black Box System Optimization:
- SOURCES: Brooks [13] application of a heuristic search method, more examples Pinter [106]

Thm 1: No algorithm with finite convergence for Problem P.
Crl 1: NO alg for Prb P have finite convergence unless it uses the EXACT value of the Lipschitz constant L to bound fn f.

usually we only have an overestimate of L. but even then, only a few Lipschitz functions can be optimized in finite steps.

if general Lipschitz functions are considered: only a point with a globally optimal value can be found with a finite number of function evaluations.

To find L, typically use sawtooth cover of f and iteratively try to approx (or overest)

Assume we know Lipschitz constant.

#### 4.2 Known Lipschitz Constant

> If the Lipschitz constant L is known but is large, the time required for solving Problem P' may become prohibitive even with just a few variables. Increasing c, i.e., giving up precision may even not be sufficient to get quickly a solution.

Other possibilites:
- depth first search to get rapidly good solution 
- generate points randomly, seelct some in high regions of the upper bounding function 
- use approximate upper bounding function obtained by considering only a few neighbouring points of each evaluation point
- estimation of local lipschitz constants in most expensive to explore regions

Global optimization is 'extremely difficult to solve' ... 

> For them not to be intractable some minimal assumptions have to be made (such as bounded slopes)

### Trajectory Methods in Global Optimization

> Consider problem (P) on Rn. The idea of a homotopy method is to consider instead of the difficult problem F(x) = 0 an appropriate easier problem G(x) = 0 which can be solved. Suppose the zeros of G are xi, ... , xk. Then one slowly "deforms" G into F and tries to follow the changes that the solutions to G(x) = 0 undergo during the deformation. 

> In otw, one introduces a new parameter t in [0,1] and defines a homotopy from G to F, a differentiable map H: Rn x [0,1] --> Rn


TAKEAWAYS...

It has been a few hours of combing through the Global Optimization handbook and my problem isn't to find all the solutions: it is to, from ANY location, find a SINGLE solution and stop.

## Trust Region based adaptive radial basis function algorithm for global optimization of expensive constrained black box problems 

- unknown function f, complex constraints, severly limited computational budget

constrained problem turned into an unconstrained problem by using a penalty function

TARBF: 
- exploit a local surrogate model in the trust region in each iteration INSTEAD of building a global surrogate model

trust region adaptively adjusted in each iteration based on four indicators:
- size of current trust region
- location of design var in trust region and in global design space
- movement history of each design variable
- iteration point vector

Okay... so the main purpose is to see if we can get away with using n constant jacobian for a certain point, and we should be able to decide upon n beforehand.

Then we found the Kantorovich guarantee for certain points that a constant jacobian would work (semilocal convergence conditions) but there are two problems:
- it is really (REALLY) conservative
- if we don't converge, what should we do to keep going? First push ourselves out of singularity to a certain extent, then keep moving with constant jacobian until... until when? 


# July 22

Roadblock:
- how useful is the Kantorovich guarantee?

What if instead we...

say if h < 1/2, only use that 1 jacobian...
now we need to see 2 plots:
- h vs number of iterations to converge successfully (if not successful then do not show)
- h vs successful

It looks like when the condition number is the lower bound for that region, we don't converge. It is really messy and seems unpredictable, because at a certain point this doesn't hold anymore, and the values still converge despite having a very high h value.

This doesn't have a lot of direction right now.

# July 23 

What is the relationship between the lipschitz constant and the condition number?

Based on Kantorovich we can see that a single jacobian is sufficient for some conditions. But based on Dylan's paper we see that the region of convergence is much bigger than Kantorovich's theorem suggests.

Let's compare the scatterplots:
- KANTOROVICH RADIUS: For all points and the same desired point, compute h. If h <= 1/2, color yellow. If h > 1/2, purple
- TEST: Compute the actual invkin for all points and see if they converge.

Maybe the first newton step can be multiplied by some factor to encourage a larger radius.

Some things to note for the Kantorovich conditions:

Most of the time, if b is < 1.5 AND the spectral radius is within a factor of 1/2 or 2 of B (so... basically just the two factors that make the conditioin number) then we seem to converge more ofen.

# July 24

Assume we KNOW bounds for the lipschitz constant, h, b, etc. 

Then we have the inequalities/expressions: 
*some condition on f or x0* <= *some predetermined constant*

> Why do we need to make these assumptions, when the Kantorovich theorem exists? ANSWER: The Kantorovich theorem is TOO CONSERVATIVE, and for the inverse kinematics problem, the function is generally far smoother than most nonlinear systems of equations in literature, so the lipschitz constant of J is actually never drastically different than the spectral norm of J at any point. 

Then we can make a bit of a sweeping statement: 
If our x0 and f fulfill *some conditions*, it's highly likely we will converge with 1 jacobian.

If **NOT** fulfill *some conditions*, it's okay:
First we ask, 

WHICH conditions are unfulfilled?

When we take a look at something like the jacobian... how can we see which joint is responsible for causing the singularity...?

For instance if we have Dylan's 3DOF arm at position q=[0,0,0] then we have jacobian of:
    j1  j2  j3
x [[ 0,  0,  0]
y  [.85, 0,  0]
z  [0, .85, .3]]

If B is very high, we are likely SINGULAR: look at the jacobian and identify which joint is very singular... so we get the jacobian...  
And it is OK if we are wrong because if we find we are going in the wrong direction.

ChatGPT's suggestion is the SVD , get the smallest singular value, and then get the corresponding right singular vector in V^T to give a direction in joint space that moves along null space (ie doesnt affect task space motion much) --> thus move orthogonal to the null space by moving in direction of vmin

But im not really sure about singulaerity avoidance so we can save this for later.

FOR NOW:
Dont start at the singular position, so we only get to the singular position when we are movign through the trajectory.

If b is very high but we aren't singular, then what should we do... How about for now, we just compute the inverse kinematics as usual, and when we find that the error is increasing, we recalibrate? Is it possible to discern how many steps are needed from the beginning, just based on b, B, and the lipschitz constant?

i try to find a lot of avoid singularity, but i think not worth it to dive too deep into it right now, since i have other method to work on... 


## SOME QUESTIONS FOR MYSELF TO REFLECT ON....

*Why can't we use the mesh refinement method?*

*Why is it valid to suppose we can even get bounds for the modified kantorovich method we are currently working on?*

*Why are we focusing on this online method and not calibrating beforehand, offline?*
- If we calibrate beforehand, it is true we can store goal agnostic information: we can store jacobians, joint position, task space position. From this, we can get the true global lipschtiz value.
- However, if we use and interpolate Jacobian related information, then... we might as well interpolate the closest joint angles that corrwspond to the desired task space position--thus solving the problem by just interpolating points and MOVINg there, instead of actually computing inverse kinematics. 
- Therefore, it seems more reasonable to focus on online strategies.

This is a difficult problem for sure... If we want to know how many jacobians are needed in general... 

Right now the issues are that we are exceeding the joint limits by a lot. How can we tackle this? By dynamic dampening? I fear we have lost the plot a little...

We know that the greater the task space error, also the more jacobians are needed.

When we get the jacobian, we can get the units for task space.

Maybe we could look at the norm of the error vector.

This is getting a bit bleak.

# July 25

Kantorovich was too good to be true...

What if instead of finding out some way to perform the inverse kinematics, we use multiple methods to try and divvy up the cartesian space into multiple jacobian-update sections for one goal?

For instance, have a desired cartesian position,
and then run several methods like the 'backwards (aka goal to initial) jacobian/trajectory planning' thing, the kantorovich predictor thing, we can run a mesh refinement... Some of these methods will ASSUME we know the function f, or the solution, etc. This step is simply to run different kinds of visualizations to show.... how many jacobians are REALLY needed? In an actual alborithm where we don't have access to sa lot of data and only bounds, can we get a prediction that is similar to the estimates where we do have a lot of data? 

*But dont forget we then have to deal with singularity avoidance and constraints and this is a whole other issue*

After we get these estimates, we can say something like....

"It should only take maximum X jacobian updates to get to get ANYWHERE!"

And from there, maybe we can use this X as kind of a... upper bound...? For any system, use a few more jacobians at the beginning and then deign to update in the middle and then more at the end 

I've also been avoiding offline pre-calibrating because of the issue where, if we access the joints/task point, we could just interpolate joint that corresponds to task space and GO THERE but we can try this to get a mapping of the joint space complexity...


## How can we put together a paper?

**Most of the complexity comes from the forward kinematics, assume we know nothing about the camera calibration**

Lipschitz constant of fkin is probably very similar to the lipschitz const of the entire VS system -can argue based on empirical evidence, show for simple cases --> the complexity is the same, theory holds, supports reasoning for KNOWING forward kin operator even if we dont know camera operator.

DURableVS: Data-efficient Unsupervised Recalibrating Visual
Servoing via online learning in a structured generative model

Perhaps this breaks when the camera uses different error constraints or when the camera is positioned weirdly, but generally it should be OK!

Units might be screwed

for 'new method and im comparing it to this common technique' papers, compare to ... broyden update method? *insert a commonly used method* ... compare to method that recomputes jacobian after every single step (the best possible method) VS based on my radius bound method --> show that we dont need these jacobians.

then analysis: situations when i need a lot more jacobian updates than this other situation, which is relatively easy... set up weird case with multiple cameras, weird constraints, show that we need a ton of jac updates since this is very complex,

show that the purely theoretical approach is too conservative, so realistic implementation needs us to be a little more.. lenient...

A, B, C
pure theory, update every step, my method: more realstic (should be BEST)

Assume we know forward kinematics function and that most of the complexity comes from the forward kinematics.
Then, can we solve the homotopy...?



In our paper maybe we can go from highly theoretical to empirical and show how we use the theory plus practical tips to make a good working model:

## Paper Idea

### Intro: brief on solving systems of NL E
This paper posits that very few jacobians are actually required to solve the inverse kinematics problem. 

We develop a practical method, Some Method X, for robot visual servoing, where we have no knowledge of {the solution joint vector(s), good starting positions, the number of solutions present, the camera calibration, the true global lipschitz constant} and where we know {the current joint position, the sampled image projection, the sampled function F}.

We do this by utilizing tools from global convergence theory (lipschitz), semilocal convergence theory (kantorovich), and empirical observations unique to the robot. 

First discuss that this is a specific case study of solving a system of nonlinear equations. In general, systems of NL E is a notoriously difficult problem: to find good starting points when the solution is unknown, and the convergence basins for any starting point can be very unpredictable; relate to that image of fractals for the basins of convergence. Even the tiniest perturbation can cause us to converge to a different solution or none at all. *complicated problem, hard to study, hard to predict* 

### Remark on practicality of existing literature, especially robot trajectory generation and invkin

In addition to the general blurb on nonlinear system, what makes the vis serv invkin problem unique, challenging, different?
- Most papers in global convergence require us to know lots of information: the function F, the solution, good starting points. At times, it is not practical to calibrate the cameras (which makes F a black box function), and it is also not practical to assume knowledge of the solution or the good starting points.

### Inspiration/starting point for research question

Now...
DESPITE the terribly sensitive behaviour of the nonlinear system, there still exists a moderately large convergence region in the task space of starting points that only need 1 jacobian to converge to some specified desired point. Using fewer jacobians is a huge benefit to computation time. {CITE Dylan's paper or some Kantorovich newton method paper that refers to needing the constant jacobian}

The questions are:
- Can we increase this radius?
- Can we quantify/predict a region as needing X updates?
- If we cannot necessarily predict HOW MANY updates we will need, can we create a model that MINIMIZES the number of updates needed, almost like a greedy alg or...?

### Provide context of the mathematical equations

Now we get into the math:

... Provides context for what the nonlinear forward kinematics function looks like, and then provides context for the visual servoing forward kinematics function. 
- Write out the analytic functions and point out that the forward kinematics function is actually relatively smooth (for a system of nonlinear equations, that is) 
- The Lipschitz constant of the fkin and VS_fkin functions is (hopefully) very similar. {CITE some source and maybe the DurableVS, showing that most of the complexity in vs is from the fkin fn}
- Since the forward kinematics function is relatively smooth (especially for our revolute robot) we can approximate the lipschitz constant as the spectral norm of any current joint position * r, where r > 1. {CITE sources that try to approximate the lipschitz constant and always try to overestimate instead of underestimate} Point out that it is a huge advantage unique to the robot system that the overall function is relativley smooth so we can get a crude approximation for the lipschitz constant without paying for much at all.

### SHOW FUNCTION LANDSCAPE

For context maybe it would be beneficial simply to show the plots of the 2 and 3 dof joint space vector VS spectral norm (goal agnostic plots) and then maybe show the region of convergence for the constant jacobian...

### SHOW NUMBER OF JACOBIAN UPDATES NEEDED VIA DIFFERENT METHODS

We have a few methods of seeing roughly how many jacobians will be needed. These methods may have access to any information becasue they exist for visualization and our understanding, not for conducting the actual inverse kinematcs...... Compare the methods and make a few notes on what they mean. For instance, near singularities we need way more tiny perturbations, we might notice that for general trajectories the first jacobian encompasses a large region, while the regions get smaller and smaller for later updates. We can also try something like... 'just try to converge but if after 5 iterations the error vector grows, update the jacobian' and see how that is different.

NOTE: Dampening can really affect the success of all the methods. We should keep dampening constant throughout and then introduce dynamic dampening at the very end in our final method (but only after showing our method withou dynamic dampening, first)

### SHOW INVERSE KINEMATICS METHODS

### METHOD 1
Control method, regular newton step update every step:

Update the jacobian every single newton step. This method should converge quickly and often.

Motivation: Our Method X should converge just as often (if not more) with less jacobians, though we can expect that it will take more iterations.

### METHOD 2
Highly theoretical method, with kantorovich and HT Kung guarantees:

Assume we have camera calibration and the forward kinematics equation:
Compute the kantorovich radius, and if we can't find it, then use the HT Kung Homotopy to find a better starting point.

Hypothesis: since the radius is so small, this method will probably converge more often, but the incessantly finding a new starting point will probably result in a lot more computation. So actually this method might be far more expensive.

We need to know the analytic form of the functions solely to conduct the homotopy.

### Diagnose issues to solve in our Method X:

Failure to converge with one jacobian boils down to two main issues: singular position OR too far away.

SOLUTIONS:

Situation:  |   Singular                                        |   Not Singular
Far Away    | get out of singularity THEN take smallish steps   |   Take smallish steps
Close       | Get out of singularity, then biggish steps        |   Good Convergence

### METHOD 3
Our method, assume that we know the bounds for the Kantorovich parameters (show how to obtain the bounds after... if we can)

If we know the forward kinematics function, and we know the projected image point(s) is there a way to compute the homotopy? i'm not so sure about this one...

Should we be trying to predict the number of updates or simply minimize the number of updates? Can we say... step until this certain amount (step until the edge of this RADIUS) and then update? 

I mean, there really isn't any point in predicting the amount of jacobian updates we will need, so just go ahead and attempt it, I'd say. 

### Obtaining the bounds for Method X

### Notes on other papers...

FABRIK: forward and backward reaching inverse kinematics... does not even use jacobian updates...

Okay today... we have outlined what a potential paper could look like which is good, but I don't know how useful or good my ideas are.

- Make the kantorovich and ht kung algorithm to show that it is slow and onerous 

What about like... if the knatorovich variables are very ill then we START off with multiple jacobian updates to get better?

._.





