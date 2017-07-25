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


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

            # Define DH param symbols ![]()

            ##   alpha i-1 -> twist angle between Z i-1 and Z i along X i-1 
            alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
            ##   a i-1 -> link length between Z i-1 and Z i along X i-1
            a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
            ##   d i -> Link offset between X i-1 and X i along Z i
            d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
            
            # Joint angle symbols
            
            ## theta i -> angle  between X i-1 and X i along Z i
            theta1, theta2, theta3, theta4, theta5, theta6, theta7 = symbols('theta1:8')

            # Modified DH params
            s = {   alpha0:     0, a0:      0, d1:  0.75,
                    alpha1: -pi/2, a1:   0.35, d2:     0, theta2: theta2-pi/2,
                    alpha2:     0, a2:   1.25, d3:     0, 
                    alpha3: -pi/2, a3: -0.054, d4:   1.5,
                    alpha4:  pi/2, a4:      0, d5:     0,
                    alpha5: -pi/2, a5:      0, d6:     0,
                    alpha6:     0, a6:      0, d7: 0.303, theta7:       0       }
            
            # Define Modified DH Transformation matrix
            ##  Correction of URDF vs. DH convention

            ## 90 degree rotation about the y-axis
            R_yaxis = Matrix([[ cos(-pi/2), 0, sin(-pi/2), 0],
                        [          0, 1,          0, 0],
                        [-sin(-pi/2), 0, cos(-pi/2), 0],
                        [          0, 0,          0, 1]])
            ## 180 degree rotation about the z-axis
            R_zaxis = Matrix([[cos(pi), -sin(pi), 0, 0],
                        [sin(pi),  cos(pi), 0, 0],
                        [      0,        0, 1, 0],
                        [      0,        0, 0, 1]])
            
            R_correlation = simplify(R_zaxis * R_yaxis)
            
            ## Corrected DH convention to URDF frame
            T_corrected = simplify(T0_G * R_correlation) 


            # Create individual transformation matrices

            T0_1 = Matrix([[            cos(theta1),            -sin(theta1),            0,              a0],
                        [sin(theta1)*cos(alpha0), cos(theta1)*cos(alpha0), -sin(alpha0), -sin(alpha0)*d1],
                        [sin(theta1)*sin(alpha0), cos(theta1)*sin(alpha0),  cos(alpha0),  cos(alpha0)*d1],
                        [                  0,                   0,            0,               1]])
            T0_1 = T0_1.subs(s)

            T1_2 = Matrix([[            cos(theta2),            -sin(theta2),            0,              a1],
                        [sin(theta2)*cos(alpha1), cos(theta2)*cos(alpha1), -sin(alpha1), -sin(alpha1)*d2],
                        [sin(theta2)*sin(alpha1), cos(theta2)*sin(alpha1),  cos(alpha1),  cos(alpha1)*d2],
                        [                  0,                   0,            0,               1]])
            T1_2 = T1_2.subs(s)

            T2_3 = Matrix([[            cos(theta3),            -sin(theta3),            0,              a2],
                        [sin(theta3)*cos(alpha2), cos(theta3)*cos(alpha2), -sin(alpha2), -sin(alpha2)*d3],
                        [sin(theta3)*sin(alpha2), cos(theta3)*sin(alpha2),  cos(alpha2),  cos(alpha2)*d3],
                        [                  0,                   0,            0,               1]])
            T2_3 = T2_3.subs(s)

            T3_4 = Matrix([[            cos(theta4),            -sin(theta4),            0,              a3],
                        [sin(theta4)*cos(alpha3), cos(theta4)*cos(alpha3), -sin(alpha3), -sin(alpha3)*d4],
                        [sin(theta4)*sin(alpha3), cos(theta4)*sin(alpha3),  cos(alpha3),  cos(alpha3)*d4],
                        [                  0,                   0,            0,               1]])
            T3_4 = T3_4.subs(s)

            T4_5 = Matrix([[            cos(theta5),            -sin(theta5),            0,              a4],
                        [sin(theta5)*cos(alpha4), cos(theta5)*cos(alpha4), -sin(alpha4), -sin(alpha4)*d5],
                        [sin(theta5)*sin(alpha4), cos(theta5)*sin(alpha4),  cos(alpha4),  cos(alpha4)*d5],
                        [                  0,                   0,            0,               1]])
            T4_5 = T4_5.subs(s)
                
            T5_6 = Matrix([[            cos(theta6),            -sin(theta6),            0,              a5],
                        [sin(theta6)*cos(alpha5), cos(theta6)*cos(alpha5), -sin(alpha5), -sin(alpha5)*d6],
                        [sin(theta6)*sin(alpha5), cos(theta6)*sin(alpha5),  cos(alpha5),  cos(alpha5)*d6],
                        [                  0,                   0,            0,               1]])
            T5_6 = T5_6.subs(s)

            T6_G = Matrix([[            cos(theta7),            -sin(theta7),            0,              a6],
                        [sin(theta7)*cos(alpha6), cos(theta7)*cos(alpha6), -sin(alpha6), -sin(alpha6)*d7],
                        [sin(theta7)*sin(alpha6), cos(theta7)*sin(alpha6),  cos(alpha6),  cos(alpha6)*d7],
                        [                  0,                   0,            0,               1]])
            T6_G = T6_G.subs(s)

            T0_2 = simplify(T0_1 * T1_2)
            T0_3 = simplify(T0_2 * T2_3)
            T0_4 = simplify(T0_3 * T3_4)
            T0_5 = simplify(T0_4 * T4_5)
            T0_6 = simplify(T0_5 * T5_6)
            T0_G = simplify(T0_6 * T6_G)

            
            # Extract end-effector position and orientation from request
	    # px,py,pz = end-effector position
	    # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])
     
            # Calculate joint angles using Geometric IK method

            ## Preparation of variables: substitution with provided values in dict 's'
                         
            ##distance_j2_j3 = a2 #1.25m default
            distance_j2_j3 = a2.subs(s)
            ##krurdf210.urdf.xacro joints
            ##fixed variable: these can be moved outside the for loop later
            distance_j3_j4 = sqrt(0.96**2 + ((-0.054)-1.25)**2)
            distance_j4_j5 = sqrt(0.54**2 + 0.96**2)

            ## vertical offset is this needed?
            distance_j3_j4_d_i = abs(a3).subs(s)

            ## see the description in the write up 
            angle_j3_j4 = pi - asin(distance_j3_j4_d_i / distance_j3_j4)
            
            ##using cosine rule to calculate the middle angle between the known sides
            distance_j3_j5 = sqrt(distance_j3_j4**2 + distance_j4_j5**2 - 2*distance_j3_j4*distance_j4_j5*cos(angle_j3_j4))

            ##calculating matrix from 3 to 6
            eef_position = Matrix([[px],
                                  [py],
                                  [pz]])

            ##eef_offset = d7 # 0.303m default
            eef_offset = d7.subs(s)
            eef_adjustment = Matrix([[eef_length],
                             [0],
                             [0]]) 

            R_yqr = simplify(rot_z(yaw) * rot_y(pitch) * rot_x(roll))

            ## by the definition in the lecture
            w_c = eef_position - R_yqr * eef_adjustment

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
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
