## Project: Kinematics Pick & Place

---


**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 


[//]: # (Image References)

[image1]: ./misc_images/misc1.png
[image2]: ./misc_images/ik.png
[image3]: ./misc_images/misc2.png
[image4]: ./misc_images/calc.jpeg

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

Schematic of reference frames for kuka arm:<br><br>
<img src="https://raw.githubusercontent.com/shbosh/rnd-project2/master/misc_images/schematic.png" />
<br>

#### Definitions of DH parameters

Twist angle, alpha(i-1): angle between axis Z(i-1) and Z(i) measured about axis X(i-1)

Link length, a: distance from axis Z(i-1) to Z(i) measured along axis X(i-1)

Link offset, d: distance from axis X(i-1) to X(i) measured along axis Z(i)

Joint angle, theta: angle between axis X(i-1) and X(i) measured about axis Z(i)

#### Relative location of joints (URDF File)

Joint | x | y	| z | axis
--- | --- | --- | --- | ---
1 | 0 | 0 | 0.33 | z
2 | 0.35 | 0 | 0.42 | y
3 | 0 | 0| 1.25 | y
4 | 0.96 | 0 | -0.54 | x
5 | 0.54 | 0 | 0 | y
6 | 0.193 | 0	| 0 | x
G | 0.11 | 0 | 0 | y

#### DH Matrix
Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | q1 
1->2 | -pi/2 | 0.35 | 0 | -pi/2 + q2
2->3 | 0 | 1.25 | 0 | q3
3->4 |  -pi/2 | -0.054 | 1.5 | q4
4->5 | pi/2 | 0 | 0 | q5
5->6 | -pi/2 | 0 | 0 | q6
6->EE | 0 | 0 | 0.303 | 0

<br>

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

Individual transformation matrices about each joint:<br>
```python

def TF_Matrix(alpha, a, d, q):
        TF = Matrix([
            [           cos(q),           -sin(q),           0,             a],
            [sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
            [sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
            [                0,                 0,           0,             1]
        ])
        return TF

# Create individual transformation matrices
T0_1 = TF_Matrix(alpha0, a0, d1, q1).subs(DH_Table)
T1_2 = TF_Matrix(alpha1, a1, d2, q2).subs(DH_Table)
T2_3 = TF_Matrix(alpha2, a2, d3, q3).subs(DH_Table)
T3_4 = TF_Matrix(alpha3, a3, d4, q4).subs(DH_Table)
T4_5 = TF_Matrix(alpha4, a4, d5, q5).subs(DH_Table)
T5_6 = TF_Matrix(alpha5, a5, d6, q6).subs(DH_Table)
T6_EE = TF_Matrix(alpha6, a6, d7, q7).subs(DH_Table)
    
```
The generalized homogeneous transform between base_link and gripper_link is:<br>
```
T0_EE = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T6_EE
```
Two additional rotations are applied to the gripper frame to correct discrepancy:<br>
```
 Rot_Error = ROT_z.subs(y, radians(180)) * ROT_y.subs(p, radians(-90))
 ```
<br>

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

##### Inverse position kinematics problem:
Joint 1, 2, 3 determine the position of the end effector. <br>
![image2]
![image4]

```python
# Calculate joint angles using Geometric IK method
theta1 = atan2(WC[1], WC[0])

# SSS triangle for theta2 and theta3
side_a = 1.501
side_b = sqrt(pow((sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35), 2)
                + pow((WC[2] - 0.75), 2))
side_c = 1.25

angle_a = acos((side_b * side_b + side_c * side_c - side_a * side_a) / (2 * side_b * side_c))
angle_b = acos((side_a * side_a + side_c * side_c - side_b * side_b) / (2 * side_a * side_c))
angle_c = acos((side_a * side_a + side_b * side_b - side_c * side_c) / (2 * side_a * side_b))

theta2 = pi / 2 - angle_a - atan2(WC[2] - 0.75, sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35)
theta3 = pi / 2 - (angle_b + 0.036) # 0.036 accounts for sag in link 4 of -0.054m
```

##### Inverse orientation kinematics problem: 
Joint 4, 5, 6 determine the orientation of the end effector. <br>
Theta4,5 and 6 can be found by transforming the orientation into euler angles using theta1,2 and 3. 

```python
# Euler angles from rotation matrix
theta4 = atan2(R3_6[2,2], -R3_6[0,2])
theta5 = atan2(sqrt(R3_6[0,2]*R3_6[0,2] + R3_6[2,2]*R3_6[2,2]), R3_6[1,2])
theta6 = atan2(-R3_6[1,1], R3_6[1,0])
```
<br>

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 

First, I defined the DH transformation matrix and used it to create the transformation matrices for each joint via forward kinematics. In order to correct the discrepancy on the gripper link, I applied a 180 degree rotation along z axis and a 90 degree rotation along the y axis, on the total transformation matrix.  Lastly, I extracted the end-effector position and orientation from the request and calculated the joint angles via inverse kinematics.<br>
<b>Results:</b> The robot arm is able to complete 10/10 pick and place cycles as shown below.<br><br>

<img src="https://raw.githubusercontent.com/shbosh/rnd-project2/master/misc_images/results.png" />
