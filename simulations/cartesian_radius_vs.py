'''
June 30 2025

- Have a camera project the robot into 2D image
- draw on projection and remember lines: OpenCV to draw circles around the radius of each joint (We might have to select the joints ourselves using ginput, and that's okay. It's not a lot of work for the user.)
- Create an intermediate goal for the last JOINT:
    (NOT to be confused with the end effector itself. For example, for a 2-DOF robot, the joint in question would be the elbow.)
    The elbow is the center of a circle. The radius of this circle is == the length of the end effector link.
    Compute the shortest distance between the goal projection and the radius of the circle (== the point where the circle and the line from the circle's center to the desired point INTERSECT)
    If that shortest distance is < NEAR_TOL (the end effector is near enough to the desired point tolerance) then we likely only need ONE jacobian.
    Else (the distance is far) then try: 
    CONSTRUCT POSE:
        
'''