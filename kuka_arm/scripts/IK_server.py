#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *

def rot_x(q):
    R_x = Matrix([[1,      0,       0],
                  [0, cos(q), -sin(q)],
                  [0, sin(q),  cos(q)]])

    return R_x

def rot_y(q):
    R_y = Matrix([[ cos(q), 0, sin(q)],
                  [      0, 1,      0],
                  [-sin(q), 0, cos(q)]])

    return R_y

def rot_z(q):
    R_z = Matrix([[cos(q), -sin(q), 0],
                  [sin(q),  cos(q), 0],
                  [     0,       0, 1]])

    return R_z

def TF_Matrix(alpha, a, d, q):
    TF = Matrix([
        [           cos(q),           -sin(q),           0,             a],
        [sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
        [sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
        [                0,                 0,           0,             1]
    ])
    return TF

def handle_calculate_IK(req, test = 'no'):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        # Initialize service response
        joint_trajectory_list = []

        # Define DH param symbols
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')

        # Joint angle symbols
            # Defined above with DH param symbols

        # Modified DH params
        DH_Table = {
                alpha0:     0, a0:      0, d1:  0.75,
                alpha1: -pi/2, a1:   0.35, d2:     0, q2: q2-pi/2,
                alpha2:     0, a2:   1.25, d3:     0,
                alpha3: -pi/2, a3: -0.054, d4:   1.5,
                alpha4:  pi/2, a4:      0, d5:     0,
                alpha5: -pi/2, a5:      0, d6:     0,
                alpha6:     0, a6:      0, d7: 0.303, q7:       0
            }

        # define Modified DH Transformation matrix
            # Total transform defined after individual transforms

        # Create individual transformation matrices
        T0_1 = TF_Matrix(alpha0, a0, d1, q1).subs(DH_Table)
        T1_2 = TF_Matrix(alpha1, a1, d2, q2).subs(DH_Table)
        T2_3 = TF_Matrix(alpha2, a2, d3, q3).subs(DH_Table)
        T3_4 = TF_Matrix(alpha3, a3, d4, q4).subs(DH_Table)
        T4_5 = TF_Matrix(alpha4, a4, d5, q5).subs(DH_Table)
        T5_6 = TF_Matrix(alpha5, a5, d6, q6).subs(DH_Table)
        T6_EE = TF_Matrix(alpha6, a6, d7, q7).subs(DH_Table)

        T0_EE = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T6_EE

        # Correct orientation of gripper link
        # 180 degree z-axis rotation
        R_z = Matrix([[cos(pi), -sin(pi), 0, 0],
                      [sin(pi),  cos(pi), 0, 0],
                      [      0,        0, 1, 0],
                      [      0,        0, 0, 1]])

        # 90 degree y-axis rotation
        R_y = Matrix([[ cos(-pi/2), 0, sin(-pi/2), 0],
                      [          0, 1,          0, 0],
                      [-sin(-pi/2), 0, cos(-pi/2), 0],
                      [          0, 0,          0, 1]])
        R_corr = simplify(R_z * R_y)

        # Total transform with gripper orientation corrected
        T_total = simplify(T0_EE * R_corr)


        for x in xrange(0, len(req.poses)):
            rospy.loginfo("Pose: %s" % x)
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

            # Extract end-effector position from request
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z
	
	    # Extract end-effector orientation from request
            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])

            # Calculate joint angles using Geometric IK method
		
            ee_length = d7
            ee_length = ee_length.subs(DH_Table)
            # a = l2
            l2_3 = a2
            l2_3 = l2_3.subs(DH_Table)
            # b = l3 + l4 w/ adjustment
            l3_4 = 0.96 # from URDF file
            l4_5 = 0.54 # from URDF file
            l3_4_offset = abs(a3)
            l3_4_offset = l3_4_offset.subs(DH_Table)
            l3_4_angle = pi - asin(l3_4_offset / l3_4)
	
            # Cosine rule: b = sqrt(d^2 + e^2 - 2de * cos(B))
            dist3_5 = sqrt(l3_4**2 + l4_5**2 - 2*l3_4*l4_5*cos(l3_4_angle))

            # Total rotation matrix from roll-pitch-yaw data
            R_total = simplify(rot_z(yaw) * rot_y(pitch) * rot_x(roll))

            # Find wrist center position using end effector position and orientation
                # d6 = 0
                # wx = px - (d6 + l) * nx
                # wy = py - (d6 + l) * ny
                # wz = pz - (d6 + l) * nz

            ee_position = Matrix([[px],
                                  [py],
                                  [pz]])

            ee_adj = Matrix([[ee_length],
                             [0],
                             [0]])

            WC = ee_position - R_total * ee_adj

            # theta1 calculation
            theta1 = atan2(WC[1,0], WC[0,0])
	
	    # SSS triangle for theta2 and theta3
	    side_a = 1.501
	    side_b = sqrt(pow((sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35), 2) + pow((WC[2] - 0.75), 2))
	    side_c = 1.25

	    angle_a = acos((side_b * side_b + side_c * side_c - side_a * side_a) / (2 * side_b * side_c))
	    angle_b = acos((side_a * side_a + side_c * side_c - side_b * side_b) / (2 * side_a * side_c))
            angle_c = acos((side_a * side_a + side_b * side_b - side_c * side_c) / (2 * side_a * side_b))

	    theta2 = pi / 2 - angle_a - atan2(WC[2] - 0.75, sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35)
	    theta3 = pi / 2 - (angle_b + 0.036) # 0.036 accounts for sag in link 4 of -0.054m

            # 6. Find R3_6 from orientation data
            R0_3 = T0_1[0:3,0:3] * T1_2[0:3,0:3] * T2_3[0:3,0:3]
            R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})

            # Correct orientation
            R_total_adjust = simplify(R_total * rot_z(-pi/2) * rot_y(-pi/2))
            # for a valid rotation matrix the transpose is == to the inverse
            R3_6 = simplify(R0_3.T * R_total_adjust)

            # theta4,5,6 calculation
            alpha, beta, gamma = tf.transformations.euler_from_matrix(R3_6.tolist(), 'ryzy')
            theta4 = alpha
            theta5 = beta
            theta6 = gamma

            # Populate response for the IK request
	    joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
	    joint_trajectory_list.append(joint_trajectory_point)
	
	rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
