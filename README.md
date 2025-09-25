# robotarm_complexity_nserc2025

### Summer 2025 Research Project, funded by the Natural Sciences and Engineering Research Council of Canada Undergraduate Student Research Award (NSERC USRA)

### This research was conducted with the University of Alberta’s Computer Vision and Robotics Research Group, supervised by Dr Martin Jagersand.

#### Research Objectives and Questions:
- How does the visual servoing function change in complexity between different degree-of-freedom robots?
- Perform adaptive mesh refinement of the visual servoing function
- Where does most of the complexity of the visual servoing function come from? (Ie the forward kinematics, the number of DOFs, the camera setup, the task constraints, etc?)
- Rather than continually update the Jacobian during Newton's Method, can we use Chord or Broyden updates to reuse that Jacobian? How can we ensure that the Chord/Broyden Jacobian is correct?

#### Research Methods:
- Using MATLAB and Python to simulate inverse kinematics, pinhole cameras, and visual servoing with different DOF robots.
- Analyzing the spectral radius of the jacobian of A during Newton's Method, where A = q - B*e, q is the current joint configuration of the robot, B is the inverse Jacobian, and e is the task space error between the desired goal and the end effector of the current joint configuration.
- Analyzing the basins of convergence

 
