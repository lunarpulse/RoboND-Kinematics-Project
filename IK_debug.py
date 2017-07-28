from sympy import *
from time import time
from mpmath import radians
import tf

'''
Format of test case is [ [[EE position],[EE orientation as quaternions]],[WC location],[joint angles]]
You can generate additional test cases by setting up your kuka project and running `$ roslaunch kuka_arm forward_kinematics.launch`
From here you can adjust the joint angles to find thetas, use the gripper to extract positions and orientation (in quaternion xyzw) and lastly use link 5
to find the position of the wrist center. These newly generated test cases can be added to the test_cases dictionary.
'''

test_cases = {1:[[[2.16135,-1.42635,1.55109], [0.708611,0.186356,-0.157931,0.661967]], [1.89451,-1.44302,1.69366], [-0.65,0.45,-0.36,0.95,0.79,0.49]],
              2:[[[-0.56754,0.93663,3.0038], [0.62073, 0.48318,0.38759,0.480629]], [-0.638,0.64198,2.9988], [-0.79,-0.11,-2.33,1.94,1.14,-3.68]],
              3:[[[-1.3863,0.02074,0.90986], [0.01735,-0.2179,0.9025,0.371016]], [-1.1669,-0.17989,0.85137], [-2.99,-0.12,0.94,4.06,1.29,-4.12]],
              4:[],
              5:[]}

def createTMatrix(alpha, a, d, q ):

    T = Matrix([[           cos(q),           -sin(q),           0,             a],

                [sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],

                [sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],

                [                0,                 0,           0,             1]])

    return(T)

def test_code(test_case):
    ## Set up code
    ## Do not modify!
    x = 0
    class Position:
        def __init__(self,EE_pos):
            self.x = EE_pos[0]
            self.y = EE_pos[1]
            self.z = EE_pos[2]
    class Orientation:
        def __init__(self,EE_ori):
            self.x = EE_ori[0]
            self.y = EE_ori[1]
            self.z = EE_ori[2]
            self.w = EE_ori[3]

    position = Position(test_case[0][0])
    orientation = Orientation(test_case[0][1])

    class Combine:
        def __init__(self,position,orientation):
            self.position = position
            self.orientation = orientation

    comb = Combine(position,orientation)

    class Pose:
        def __init__(self,comb):
            self.poses = [comb]

    req = Pose(comb)
    start_time = time()
    
    ########################################################################################
    ## 

    ## Insert IK code here!
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
    R_yaxis = R_y_DH_URDF.row_join(Matrix([[0],[0],[0]])).col_join(Matrix([[0,0,0,1]]))
    R_zaxis = R_z_DH_URDF.row_join(Matrix([[0],[0],[0]])).col_join(Matrix([[0,0,0,1]]))
    
    R_correction = R_zaxis * R_yaxis
    
    # Create individual transformation matrices !(relative translation and orientation of link i-1 to link i)[https://d17h27t6h515a5.cloudfront.net/topher/2017/May/592d6644_dh-transform-matrix/dh-transform-matrix.png]

    T0_1 = createTMatrix(alpha0, a0, d1, quaternion1 ).subs(s)
    T1_2 = createTMatrix(alpha1, a1, d2, quaternion2 ).subs(s)
    T2_3 = createTMatrix(alpha2, a2, d3, quaternion3 ).subs(s)
    T3_4 = createTMatrix(alpha3, a3, d4, quaternion4 ).subs(s)
    T4_5 = createTMatrix(alpha4, a4, d5, quaternion5 ).subs(s)
    T5_6 = createTMatrix(alpha5, a5, d6, quaternion6 ).subs(s)
    T6_EEF = createTMatrix(alpha6, a6, d7, quaternion7 ).subs(s)

    T0_2 = T0_1 * T1_2
    T0_3 = simplify(T0_2 * T2_3)
    T0_4 = T0_3 * T3_4
    T0_5 = T0_4 * T4_5
    T0_6 = T0_5 * T5_6 
    #simplify only T0_EFF and T0_3 as others are intermediate calculations no need of simplification in optimistaion phase
    T0_EEF = T0_6 * T6_EEF



    ## Corrected DH convention to URDF frame
    T_corrected = simplify(T0_EEF * R_correction) 
    # Extract end-effector position and orientation from request

    # px,py,pz = end-effector position
    # roll, pitch, yaw = end-effector orientation
    px = req.poses[x].position.x
    py = req.poses[x].position.y
    pz = req.poses[x].position.z

    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
        [req.poses[x].orientation.x, req.poses[x].orientation.y,
            req.poses[x].orientation.z, req.poses[x].orientation.w])
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
    wrist_centre = eef_position -  R_ypr.subs({'r' : roll, 'p' : pitch,  'y': yaw}) * eef_adjustment
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

    ## R_rpy = R_ypr
    R0_3 = T0_3[0:3,0:3]
    R0_3 = R0_3.evalf(subs={quaternion1: theta1, quaternion2: theta2, quaternion3: theta3})

    ## the inverse matrix cancel the first three rotations
    R3_6 = R0_3.inv() * R_ypr_adjusted
    
    print ("\n T0_3:")
    print(T0_3)
    print ("\n R3_6:")
    print(R3_6)

    ## Find iota, kappa, zetta euler angles as done in lesson 2 part 8. ()[https://d17h27t6h515a5.cloudfront.net/topher/2017/May/591e1115_image-0/image-0.png]
    syp = sqrt(R3_6[1, 2]*R3_6[1, 2] +R3_6[1, 0]*R3_6[1, 0])

    if syp > 0.000001:
        theta6_p = atan2( R3_6[1, 2],  R3_6[1, 0])
        theta5_p = atan2( syp,       R3_6[1, 1])
        theta4_p = atan2( R3_6[2, 1], -R3_6[0, 1])
    else:
        theta6_p = atan2(-R3_6[2, 0],  R3_6[2, 2])
        theta5_p = atan2( syp,       R3_6[1, 1])
        theta4_p = 0.0


    syn = -sqrt(R3_6[1, 2]*R3_6[1, 2] +R3_6[1, 0]*R3_6[1, 0])

    theta6, theta5, theta4 = theta6_p, theta5_p, theta4_p

    ## euler_from_matrix assuming a yzy rotation (j4 : y, j5: z, j6: y)
    #theta4, theta5, theta6 = tf.transformations.euler_from_matrix(R3_6.tolist(), 'ryzy')
    print ("\n Test case: ")
    print (test_case_number)
    print ("\n theta1, theta2, theta3 ,theta4, theta5, theta6:")
    print (theta1, theta2, theta3 ,theta4, theta5, theta6)
    ## 
    ########################################################################################
    
    ########################################################################################
    ## For additional debugging add your forward kinematics here. Use your previously calculated thetas
    ## as the input and output the position of your end effector as your_ee = [x,y,z]

    ## (OPTIONAL) YOUR CODE HERE!
    end_effector = T_corrected.evalf(subs={quaternion1: theta1, quaternion2: theta2, quaternion3: theta3, quaternion4: theta4, quaternion5: theta5, quaternion6: theta6 })
    ## End your code input for forward kinematics here!
    ########################################################################################

    ## For error analysis please set the following variables of your WC location and EE location in the format of [x,y,z]
    your_wc = [wrist_centre[0],wrist_centre[1],wrist_centre[2]] # <--- Load your calculated WC values in this array
    your_ee = [end_effector[0,3],end_effector[1,3],end_effector[2,3]] # <--- Load your calculated end effector value from your forward kinematics
    ########################################################################################

    ## Error analysis
    print ("\nTotal run time to calculate joint angles from pose is %04.4f seconds" % (time()-start_time))

    # Find WC error
    if not(sum(your_wc)==3):
        wc_x_e = abs(your_wc[0]-test_case[1][0])
        wc_y_e = abs(your_wc[1]-test_case[1][1])
        wc_z_e = abs(your_wc[2]-test_case[1][2])
        wc_offset = sqrt(wc_x_e**2 + wc_y_e**2 + wc_z_e**2)
        print ("\nWrist error for x position is: %04.8f" % wc_x_e)
        print ("Wrist error for y position is: %04.8f" % wc_y_e)
        print ("Wrist error for z position is: %04.8f" % wc_z_e)
        print ("Overall wrist offset is: %04.8f units" % wc_offset)

    # Find theta errors
    t_1_e = abs(theta1-test_case[2][0])
    t_2_e = abs(theta2-test_case[2][1])
    t_3_e = abs(theta3-test_case[2][2])
    t_4_e = abs(theta4-test_case[2][3])
    t_5_e = abs(theta5-test_case[2][4])
    t_6_e = abs(theta6-test_case[2][5])
    print ("\nTheta 1 error is: %04.8f" % t_1_e)
    print ("Theta 2 error is: %04.8f" % t_2_e)
    print ("Theta 3 error is: %04.8f" % t_3_e)
    print ("Theta 4 error is: %04.8f" % t_4_e)
    print ("Theta 5 error is: %04.8f" % t_5_e)
    print ("Theta 6 error is: %04.8f" % t_6_e)
    print ("\n**These theta errors may not be a correct representation of your code, due to the fact \
           \nthat the arm can have muliple positions. It is best to add your forward kinmeatics to \
           \nconfirm whether your code is working or not**")
    print (" ")

    # Find FK EE error
    if not(sum(your_ee)==3):
        ee_x_e = abs(your_ee[0]-test_case[0][0][0])
        ee_y_e = abs(your_ee[1]-test_case[0][0][1])
        ee_z_e = abs(your_ee[2]-test_case[0][0][2])
        ee_offset = sqrt(ee_x_e**2 + ee_y_e**2 + ee_z_e**2)
        print ("\nEnd effector error for x position is: %04.8f" % ee_x_e)
        print ("End effector error for y position is: %04.8f" % ee_y_e)
        print ("End effector error for z position is: %04.8f" % ee_z_e)
        print ("Overall end effector offset is: %04.8f units \n" % ee_offset)




if __name__ == "__main__":
    # Change test case number for different scenarios
    test_case_number = 1

    test_code(test_cases[test_case_number])
