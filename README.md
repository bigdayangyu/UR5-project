# UR5-project

The Objective of the project is to use the UR5 robot to complete a "place-and-draw" task using inverse kinematics joint angle control, using diﬀerential kinematics rate control, as well as Gradient-based control. 

When the program starts, user is prompted by the program to “teach” the robot by manually moving the end effector (EE) and marker to the start and end points of each line. When the user indicates that the robot is in position, the algorithm captures the current joint angles using API ur5.get_current_joints() before prompting the user for the next point. This continues until all 4 start and end points are collected. Based off of the start and end points of each line, the program generate intermediate points and using selected control method to calculate the forward kinematics of the robot, and then generate the inverse kinematics to move the robot to the desired location.

ur5_project.m file allow user to load a image with simple outline, and searches for boundaries of the image to generate image outline. The program will then convert pixel values into x and y coordinates so the robot can draw the selected image onto a paper. 

### Methods
- InvKinControl.m: <p>The inverse kinematic control algorithm uses the start and target joint angles and forward kinematics to calculate the start and target poses as homogeneous transformation matrices. Using these poses, it then computes intermediate points or waypoints that divide both the change in position and the change in orientation evenly over the straight line path connecting the start and target poses. </p>
- ur5RateControl.m  <p> Uses differential kinematics to derive the relationship between the joint displacement and the end-effecter location, and then find the right direction to change joint angles(joint space velocity). </p>
- ur5GradControl.m <p>Gradient Control algorithm is identical to that of the Rate Control algorithm: finding the diﬀerence between the current transformation and the desired transformation and calculate the transpose of the Jacobian, multiply by control gain k to find the desired joint velocities and its direction. </p>
- ur5_project.m <p>Detect image boundary and convert the boundary into coordinate in the robot workspace, and then use inverse kinematic control to control the robot to draw the image onto a paper.  </p>
