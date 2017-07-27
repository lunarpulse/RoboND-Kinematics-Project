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

def createTMatrix (alpha, a, d, q):

    T = Matrix([[           cos(q),           -sin(q),           0,             a],

                [sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],

                [sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],

                [                0,                 0,           0,             1]])

    return(T)

def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

        # Define DH param symbols ![]()
        ##   alpha i-1 -> twist angle between Z i-1 and Z i along X i-1 
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
        ##   a i-1 -> link length between Z i-1 and Z i along X i-1
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
        ##   d i -> Link offset between X i-1 and X i along Z i
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        
        # Joint angle symbols
        
        ## quaternion i -> angle  between X i-1 and X i along Z i
        quaternion1, quaternion2, quaternion3, quaternion4, quaternion5, quaternion6, quaternion7 = symbols('quaternion1:8')

        # Modified DH params
        s = {   alpha0:     0, a0:      0, d1:  0.75,
                alpha1: -pi/2, a1:   0.35, d2:     0, quaternion2: quaternion2-pi/2,
                alpha2:     0, a2:   1.25, d3:     0, 
                alpha3: -pi/2, a3: -0.054, d4:   1.5,
                alpha4:  pi/2, a4:      0, d5:     0,
                alpha5: -pi/2, a5:      0, d6:     0,
                alpha6:     0, a6:      0, d7: 0.303, quaternion7:       0       }
        
        # Define Modified DH Transformation matrix
        ##  Correction of URDF vs. DH convention
        r,p,y = symbols('r p y')

        Rot_x = Matrix([[1,         0,          0], 
                        [0, cos(r), -sin(r)],
                        [0, sin(r),  cos(r)]])

        Rot_y = Matrix([[ cos(p), 0, sin(p)],
                        [          0, 1,          0],
                        [-sin(p), 0, cos(p)]])

        Rot_z = Matrix([[cos(y), -sin(y), 0], 
                        [sin(y),  cos(y), 0],
                        [     0,       0,     1]])
        R_y_DH_URDF = Rot_y.subs(p, -pi/2)
        R_z_DH_URDF = Rot_z.subs(y, -pi/2)
        R_correction_convention = R_z_DH_URDF * R_y_DH_URDF
                    
        ## 90 degree rotation about the y-axis
        R_yaxis = Rot_y.subs(p, -pi/2).row_join(Matrix([[0],[0],[0]])).col_join(Matrix([[0,0,0,1]]))
        R_zaxis = Rot_z.subs(p, pi).row_join(Matrix([[0],[0],[0]])).col_join(Matrix([[0,0,0,1]]))
        
        R_correction = R_zaxis * R_yaxis
        
        # Create individual transformation matrices !(relative translation and orientation of link i-1 to link i)[https://d17h27t6h515a5.cloudfront.net/topher/2017/May/592d6644_dh-transform-matrix/dh-transform-matrix.png]

        T0_1 = createTMatrix(alpha0, a0, d1, quaternion1 ).subs(s)
        T1_2 = createTMatrix(alpha1, a1, d2, quaternion2 ).subs(s)
        T2_3 = createTMatrix(alpha2, a2, d3, quaternion3 ).subs(s)
        #T3_4 = createTMatrix(alpha3, a3, d4, quaternion4 ).subs(s)
        #T4_5 = createTMatrix(alpha4, a4, d5, quaternion5 ).subs(s)
        #T5_6 = createTMatrix(alpha5, a5, d6, quaternion6 ).subs(s)
        #T6_EEF = createTMatrix(alpha6, a6, d7, quaternion7 ).subs(s)

        T0_2 = T0_1 * T1_2
        #simplify only T0_EFF and T0_3 as others are intermediate calculations no need of simplification in optimistaion phase
        T0_3 = simplify(T0_2 * T2_3)
        #T0_4 = T0_3 * T3_4
        #T0_5 = T0_4 * T4_5
        #T0_6 = T0_5 * T5_6 
        #T0_EEF = T0_6 * T6_EEF

        ## Corrected DH convention to URDF frame
        #T_corrected = simplify(T0_EEF * R_correction) 
        
        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()



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

                         
            ## Calculating Theta

            ## Preparation of variables: substitution with provided values in dict 's'

            ##calculating matrix from 3 to 6
            eef_position = Matrix([[px],
                                   [py],
                                   [pz]])

            ##distance_j5_eef = d7 + d6 # 0.303m + 0 m urdf from j 5-6-EEF along x axis
            distance_j5_eef = d7.subs(s)+ d6.subs(s)
            #align on x axis as defined in urdf.zacro file
            eef_adjustment = Matrix([[distance_j5_eef],
                                     [0],
                                     [0]]) 

            R_ypr = Rot_z * Rot_y * Rot_x
            
            ## Correct orientation between DH convention and URDF 

            R_ypr_adjusted = R_ypr * R_correction_convention
            R_ypr_adjusted = R_ypr_adjusted.subs({'r': roll, 'p': pitch, 'y': yaw})
            
            ## by the definition in the lecture //!()[https://d17h27t6h515a5.cloudfront.net/topher/2017/May/592d74d1_equations/equations.png]
            wrist_centre = eef_position - R_ypr.subs({'r' : roll, 'p' : pitch,  'y': yaw}) * eef_adjustment

            #calculating theta1 from atan2(y_c, x_c)
            theta1 = atan2(wrist_centre[1,0], wrist_centre[0,0])

            ##distance_j2_j3 = a2 #1.25m default
            distance_j2_j3 = a2.subs(s)
            ## from krurdf210.urdf.xacro joints
            distance_j3_j4 = sqrt(0.96**2 + (-0.054)**2)
            distance_j4_j5 = 0.54

            ## vertical offset di
            distance_j3_j4_d_i = abs(a3.subs(s))

            ## see the description in the write up 
            angle_j3_j4 = pi - asin(distance_j3_j4_d_i / distance_j3_j4)
            
            ##using cosine rule to calculate the middle angle between the known sides
            distance_j3_j5 = sqrt(distance_j3_j4**2 + distance_j4_j5**2 - 2*distance_j3_j4*distance_j4_j5*cos(angle_j3_j4))

            ## preparing to calculate theta2 in relation to j2 to j5
            j2_x = a1 * cos(theta1)
            j2_x = j2_x.subs(s)

            j2_y = a1 * sin(theta1)
            j2_y = j2_y.subs(s)

            j2_z = d1.subs(s)

            j5_x = wrist_centre[0,0]
            j5_y = wrist_centre[1,0]
            j5_z = wrist_centre[2,0]
            ## diff z between joint 2 and 5
            j5_z_j2_z = j5_z - j2_z

            ## hypenetus of triangle with points of j2, j3, j5.
            distance_j2_j5 = sqrt((j5_x - j2_x)**2 + (j5_y - j2_y)**2 + (j5_z - j2_z)**2)
            ## distance j2_j5 superimposed on xy plane
            distance_j2_j5_on_xy = sqrt((j5_x - j2_x)**2 + (j5_y - j2_y)**2)

            # theta2 = pi/2 - beta - eta
            beta = acos((distance_j3_j5**2 - distance_j2_j3**2 - distance_j2_j5**2)/(-2*distance_j2_j3*distance_j2_j5))
            eta = atan2(j5_z_j2_z, distance_j2_j5_on_xy)
            theta2 = pi/2 - beta - eta

            ##  theta3 calc
            ## theta3 = pi/2 - delta - gamma
            delta = asin(distance_j3_j4_d_i/distance_j3_j5) #asin(a3/b)
            gamma = acos((distance_j2_j5**2 - distance_j2_j3**2 - distance_j3_j5**2)/(-2*distance_j2_j3*distance_j3_j5))#acos((c^2 - a^2 - b^2)/(-2ab))
            theta3 = pi/2 - delta - gamma
		
            ## Find R3_6 from orientation data

            R0_3 = T0_3[0:3,0:3]
            R0_3 = R0_3.evalf(subs={quaternion1: theta1, quaternion2: theta2, quaternion3: theta3})

            ## the inverse matrix cancel the first three rotations
            R3_6 = R0_3.inv() * R_ypr_adjusted

            ## Find iota, kappa, zetta euler angles as done in lesson 2 part 8. ()[https://d17h27t6h515a5.cloudfront.net/topher/2017/May/591e1115_image-0/image-0.png]

            ## euler_from_matrix assuming a yzy rotation (j4 : y, j5: z, j6: y)
            iota, kappa, zetta = tf.transformations.euler_from_matrix(R3_6.tolist(), 'ryzy')
            theta4 = iota
            theta5 = kappa
            theta6 = zetta

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
