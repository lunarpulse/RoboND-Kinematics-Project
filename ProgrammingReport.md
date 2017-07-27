## Project: Kinematics Pick & Place
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---


**Steps to complete the project:**  


- [x] Set up your ROS Workspace. 
- [x] Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
- [x] Experiment with the forward_kinematics environment and get familiar with the robot.
- [x] Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
- [x] Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
- [x] Fill in the `IK_server.py` with your Inverse Kinematics code. 


[//]: # (Image References)
[image0]: https://d17h27t6h515a5.cloudfront.net/topher/2017/May/592d6644_dh-transform-matrix/dh-transform-matrix.png
[image1]: ./misc_images/IK_debug_resized.png
[image2]: ./misc_images/wrongPlan_resized.png
[image3]: ./misc_images/completion_resized.png
[image4]: ./misc_images/fasterCalculation_resized.png
[image5]: ./misc_images/fasterDebug.png
[image6]: ./misc_images/which-programs-are-fastest-middle.png
## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
## Kinematic Analysis
### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

Using the [kr210.urdf.xacro](./kuka_arm/urdf/kr210.urdf.xacro), the DH parameter table was filled.

0 | T | alpha *i-1* | a *i-1* | d*i* | quaternion*i*|
--- | --- | --- | --- | --- | ---|
1 | 0-1 | 0 | 0 | 0.75| quaternion*1*|
2 | 1-2 | -(pi/2)|0.35 | 0 |quaternion*2* -(pi/2)|
3 | 2-3 | 0 |  1.25| 0|quaternion*3*|
4 | 3-4 | -(pi/2) | -0.054 | 1.5|quaternion*4*|
5 | 4-5 | (pi/2) | 0 | 0|quaternion*5*|
6 | 5-6 |  -(pi/2)| 0 | 0|quaternion*6*|
7 | 6-End effector | 0 | 0 | 0.303   | None|

__joint__ tags indicated all the joint and gripper. The tricky part from this The x,y,z value are the increment values for d*i*from the previous joint and the difference and changes between DH frame and urdf frame was a confusing part to determine the a*i-1*. The lessons about this DH parameters were helpful to verify.

The DH table was coded as shown below
```python
s = {   alpha0:     0, a0:      0, d1:  0.75,
        alpha1: -pi/2, a1:   0.35, d2:     0, quaternion2: quaternion2-pi/2,
        alpha2:     0, a2:   1.25, d3:     0, 
        alpha3: -pi/2, a3: -0.054, d4:   1.5,
        alpha4:  pi/2, a4:      0, d5:     0,
        alpha5: -pi/2, a5:      0, d6:     0,
        alpha6:     0, a6:      0, d7: 0.303, quaternion7:       0       }
```
### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

The individual transformation matrix was created to figure out the overall transformation matrix for forward kinematics and also other needed transformation matrix, such as T0_3.

![transformation matrix general form][image0]

The above general form was used as a function to generate sympy muatableMatrix class
```python
def createTMatrix(alpha, a, d, quaternion ):

    T = Matrix([[           cos(quaternion),           -sin(quaternion),           0,             a],

                [sin(quaternion)*cos(alpha), cos(quaternion)*cos(alpha), -sin(alpha), -sin(alpha)*d],

                [sin(quaternion)*sin(alpha), cos(quaternion)*sin(alpha),  cos(alpha),  cos(alpha)*d],

                [                0,                 0,           0,             1]])

return(T)
```
With the general form above, a transformation matrix for every joint were populated. The tranformation matrix from the base link to the end effector was made with extrinsic multiplication of all the trasnfomration matrix between the components. The below is the simplified transformation matrix from base link (0) to end effector with DH parameter applied. It is a complex and lengthy matrix, which requires heavy computation to process later in the loop. 


```python
T0_EEF = Matrix([
[((sin(quaternion1)*sin(quaternion4) + sin(quaternion2 + quaternion3)*cos(quaternion1)*cos(quaternion4))*cos(quaternion5) + sin(quaternion5)*cos(quaternion1)*cos(quaternion2 + quaternion3))*cos(quaternion6) + (sin(quaternion1)*cos(quaternion4) - sin(quaternion4)*sin(quaternion2 + quaternion3)*cos(quaternion1))*sin(quaternion6), -((sin(quaternion1)*sin(quaternion4) + sin(quaternion2 + quaternion3)*cos(quaternion1)*cos(quaternion4))*cos(quaternion5) + sin(quaternion5)*cos(quaternion1)*cos(quaternion2 + quaternion3))*sin(quaternion6) + (sin(quaternion1)*cos(quaternion4) - sin(quaternion4)*sin(quaternion2 + quaternion3)*cos(quaternion1))*cos(quaternion6), -(sin(quaternion1)*sin(quaternion4) + sin(quaternion2 + quaternion3)*cos(quaternion1)*cos(quaternion4))*sin(quaternion5) + cos(quaternion1)*cos(quaternion5)*cos(quaternion2 + quaternion3), -0.303*(sin(quaternion1)*sin(quaternion4) + sin(quaternion2 + quaternion3)*cos(quaternion1)*cos(quaternion4))*sin(quaternion5) + (1.25*sin(quaternion2) + 0.35)*cos(quaternion1) - 0.054*sin(quaternion2 + quaternion3)*cos(quaternion1) + 0.303*cos(quaternion1)*cos(quaternion5)*cos(quaternion2 + quaternion3) + 1.5*cos(quaternion1)*cos(quaternion2 + quaternion3)],
[((sin(quaternion1)*sin(quaternion2 + quaternion3)*cos(quaternion4) - sin(quaternion4)*cos(quaternion1))*cos(quaternion5) + sin(quaternion1)*sin(quaternion5)*cos(quaternion2 + quaternion3))*cos(quaternion6) - (sin(quaternion1)*sin(quaternion4)*sin(quaternion2 + quaternion3) + cos(quaternion1)*cos(quaternion4))*sin(quaternion6), -((sin(quaternion1)*sin(quaternion2 + quaternion3)*cos(quaternion4) - sin(quaternion4)*cos(quaternion1))*cos(quaternion5) + sin(quaternion1)*sin(quaternion5)*cos(quaternion2 + quaternion3))*sin(quaternion6) - (sin(quaternion1)*sin(quaternion4)*sin(quaternion2 + quaternion3) + cos(quaternion1)*cos(quaternion4))*cos(quaternion6), -(sin(quaternion1)*sin(quaternion2 + quaternion3)*cos(quaternion4) - sin(quaternion4)*cos(quaternion1))*sin(quaternion5) + sin(quaternion1)*cos(quaternion5)*cos(quaternion2 + quaternion3), -0.303*(sin(quaternion1)*sin(quaternion2 + quaternion3)*cos(quaternion4) - sin(quaternion4)*cos(quaternion1))*sin(quaternion5) + (1.25*sin(quaternion2) + 0.35)*sin(quaternion1) - 0.054*sin(quaternion1)*sin(quaternion2 + quaternion3) + 0.303*sin(quaternion1)*cos(quaternion5)*cos(quaternion2 + quaternion3) + 1.5*sin(quaternion1)*cos(quaternion2 + quaternion3)],
[                                                                                                                              -(sin(quaternion5)*sin(quaternion2 + quaternion3) - cos(quaternion4)*cos(quaternion5)*cos(quaternion2 + quaternion3))*cos(quaternion6) - sin(quaternion4)*sin(quaternion6)*cos(quaternion2 + quaternion3),                                                                                                                                 (sin(quaternion5)*sin(quaternion2 + quaternion3) - cos(quaternion4)*cos(quaternion5)*cos(quaternion2 + quaternion3))*sin(quaternion6) - sin(quaternion4)*cos(quaternion6)*cos(quaternion2 + quaternion3),                                                                         -sin(quaternion5)*cos(quaternion4)*cos(quaternion2 + quaternion3) - sin(quaternion2 + quaternion3)*cos(quaternion5),                                                                                                                              -0.303*sin(quaternion5)*cos(quaternion4)*cos(quaternion2 + quaternion3) - 0.303*sin(quaternion2 + quaternion3)*cos(quaternion5) - 1.5*sin(quaternion2 + quaternion3) + 1.25*cos(quaternion2) - 0.054*cos(quaternion2 + quaternion3) + 0.75],
[                                                                                                                                                                                                                                                                                                                                      0,                                                                                                                                                                                                                                                                                                                                        0,                                                                                                                                                                                           0,                                                                                                                                                                                                                                                                                                                                                                       1]])
```
As URDF and DH conventions have a difference in rotation axis of joints and the gripper, this discrepancy must be resolved by two correction matrix, one with 2 pi rotation around z axis and the other with - pi/2 rotation on y axis.

```python
R_y_URDF_DH = Matrix([[ cos(-pi/2), 0, sin(-pi/2)],
                    [          0, 1,          0],
                    [-sin(-pi/2), 0, cos(-pi/2)]])

R_z_URDF_DH = Matrix([[cos(-pi/2), -sin(-pi/2), 0], 
                    [sin(-pi/2),  cos(-pi/2), 0],
                    [     0,       0,     1]])
```

After applying these correction matrix to the *T0_EEF*, we get **T_corrected**:
```python
T_corrected =T0_EEF * R_y_URDF_DH* R_z_URDF_DH
```

These code above are from [sympy test jupyter notebook](./kuka_arm/scripts/sympy_note.ipynb)

Although it is a requirement in rubric, T0_3 is only used in the inverse kinematics calculation.
### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

The wrist centre is the important component which enables us to isolate the problems and solve the unknown angles of each joint. The problem laid in the domain of inverse position kinematics, once setting up the joint as URDF describes.

The theory behine this is from udacity lecture note 3.14 and 2.19.

![](https://d17h27t6h515a5.cloudfront.net/topher/2017/May/592d7369_homo-xform-2/homo-xform-2.png)
![](https://d17h27t6h515a5.cloudfront.net/topher/2017/May/592d74d1_equations/equations.png)
![](https://d17h27t6h515a5.cloudfront.net/topher/2017/May/591e52e6_image-4/image-4.png)

These two snipets were implemented.
```python
eef_position = Matrix([[px],
                        [py],
                        [pz]])

distance_j5_eef = d7.subs(s)+ d6.subs(s)

eef_adjustment = Matrix([[distance_j5_eef],
                            [0],
                            [0]]) 

R_x = Matrix([[1,         0,          0], 
                [0, cos(roll), -sin(roll)],
                [0, sin(roll),  cos(roll)]])

R_y = Matrix([[ cos(pitch), 0, sin(pitch)],
                [          0, 1,          0],
                [-sin(pitch), 0, cos(pitch)]])

R_z = Matrix([[cos(yaw), -sin(yaw), 0], 
                [sin(yaw),  cos(yaw), 0],
                [     0,       0,     1]])

R_ypr = R_z * R_y * R_x

#Rzyx=  Matrix([[cos(pitch)*cos(yaw), -sin(yaw)*cos(pitch), sin(pitch)], [sin(roll)*sin(pitch)*cos(yaw) + sin(yaw)*cos(roll), -sin(roll)*sin(pitch)*sin(yaw) + cos(roll)*cos(yaw), -sin(roll)*cos(pitch)], [sin(roll)*sin(yaw) - sin(pitch)*cos(roll)*cos(yaw), sin(roll)*cos(yaw) + sin(pitch)*sin(yaw)*cos(roll), cos(roll)*cos(pitch)]])

w_c = eef_position - R_ypr * eef_adjustment
```
The distance from j5 to end effector (gripper) is the tsum of d7 and d6. the end effector was assumed to point x axis by applying orthonormal vectors with *eef_adjustment* to the homogeneous transformation formed with yaw, pitch, roll values substituted from the trajectory values from the *joint_trajectory_list*.

Inverse kinematics was solved with two similar trigometry calculations for the triangles invloving the joint 3, 4, 5 by using the wrist centre which is located on joint 5.

Theta 1, angle for joint 1 was found by solving atan2 function with y component of wrist centre and x of wrist centre, which was simply projected from the 3d point w_c.

Theta 2, and Theta 3 was solved by using the example from **Term 3. 2. 19 Inverse Kinematics Example**.


The rest of theta 4,5,6 can be found from inverse orientation kinematics using known theta 1, 2, 3. These Euler angles can be found from the rotation matrix from joint 3 to 6.

![](https://d17h27t6h515a5.cloudfront.net/topher/2017/May/591e52f0_image-5/image-5.png)

This partial rotation matrix can be derived from multiplication of T0_3.T to adjust yaw pitch roll Rotation matrix, which represent the state of endeffector. 
```python
## Correct orientation between DH convention and URDF 
R_ypr_adjusted = R_ypr* R_z_DH_URDF * R_y_DH_URDF
## R_rpy = R_ypr 
R0_3 = Matrix([[T0_3[0,0], T0_3[0,1], T0_3[0,2]],
            [T0_3[1,0], T0_3[1,1], T0_3[1,2]],
            [T0_3[2,0], T0_3[2,1], T0_3[2,2]]])
R0_3 = R0_3.evalf(subs={quaternion1: theta1, quaternion2: theta2, quaternion3: theta3})

# R0_3 = Matrix([[sin(quaternion2 + quaternion3)*cos(quaternion1), cos(quaternion1)*cos(quaternion2 + quaternion3), -sin(quaternion1)], [sin(quaternion1)*sin(quaternion2 + quaternion3), sin(quaternion1)*cos(quaternion2 + quaternion3), cos(quaternion1)], [cos(quaternion2 + quaternion3), -sin(quaternion2 + quaternion3), 0]])

## the inverse matrix cancel the first three rotations
R3_6 = R0_3.inv() * R_ypr_adjusted

#R3_6 = Matrix([[(sin(quaternion1)*sin(quaternion3) + sin(quaternion2)*cos(quaternion1)*cos(quaternion3))*sin(quaternion2 + quaternion3)*cos(quaternion1) + (sin(quaternion1)*sin(quaternion2)*cos(quaternion3) - sin(quaternion3)*cos(quaternion1))*sin(quaternion1)*sin(quaternion2 + quaternion3) + cos(quaternion2)*cos(quaternion3)*cos(quaternion2 + quaternion3), sin(quaternion1)**2*sin(quaternion2 + quaternion3)*cos(quaternion2) - sin(quaternion2)*cos(quaternion2 + quaternion3) + sin(quaternion2 + quaternion3)*cos(quaternion1)**2*cos(quaternion2), (-sin(quaternion1)*cos(quaternion3) + sin(quaternion2)*sin(quaternion3)*cos(quaternion1))*sin(quaternion2 + quaternion3)*cos(quaternion1) + (sin(quaternion1)*sin(quaternion2)*sin(quaternion3) + cos(quaternion1)*cos(quaternion3))*sin(quaternion1)*sin(quaternion2 + quaternion3) + sin(quaternion3)*cos(quaternion2)*cos(quaternion2 + quaternion3)], [(sin(quaternion1)*sin(quaternion3) + sin(quaternion2)*cos(quaternion1)*cos(quaternion3))*cos(quaternion1)*cos(quaternion2 + quaternion3) + (sin(quaternion1)*sin(quaternion2)*cos(quaternion3) - sin(quaternion3)*cos(quaternion1))*sin(quaternion1)*cos(quaternion2 + quaternion3) - sin(quaternion2 + quaternion3)*cos(quaternion2)*cos(quaternion3), sin(quaternion1)**2*cos(quaternion2)*cos(quaternion2 + quaternion3) + sin(quaternion2)*sin(quaternion2 + quaternion3) + cos(quaternion1)**2*cos(quaternion2)*cos(quaternion2 + quaternion3), (-sin(quaternion1)*cos(quaternion3) + sin(quaternion2)*sin(quaternion3)*cos(quaternion1))*cos(quaternion1)*cos(quaternion2 + quaternion3) + (sin(quaternion1)*sin(quaternion2)*sin(quaternion3) + cos(quaternion1)*cos(quaternion3))*sin(quaternion1)*cos(quaternion2 + quaternion3) - sin(quaternion3)*sin(quaternion2 + quaternion3)*cos(quaternion2)], [-(sin(quaternion1)*sin(quaternion3) + sin(quaternion2)*cos(quaternion1)*cos(quaternion3))*sin(quaternion1) + (sin(quaternion1)*sin(quaternion2)*cos(quaternion3) - sin(quaternion3)*cos(quaternion1))*cos(quaternion1), 0, -(-sin(quaternion1)*cos(quaternion3) + sin(quaternion2)*sin(quaternion3)*cos(quaternion1))*sin(quaternion1) + (sin(quaternion1)*sin(quaternion2)*sin(quaternion3) + cos(quaternion1)*cos(quaternion3))*cos(quaternion1)]])
```
 
R3_6 contains angle values for three joints, however, this is yzy rotation, due tot he orientation of revolute joints, joint 4, 5, 6. Therefore, this angles must be found in Ryzy, which is different from the [lecture note](https://d17h27t6h515a5.cloudfront.net/topher/2017/May/591e1115_image-0/image-0.png) finding Rxyz. Instead of implmenting own code for this calucation, tf package has a helper function finds the euler angles from a matrix.
```python
## euler_from_matrix assuming a yzy rotation (j4 : y, j5: z, j6: y)
iota, kappa, zetta = tf.transformations.euler_from_matrix(R3_6.tolist(), "ryzy")
```

## Project Implementation

### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 

Most code had been introduced through aforementioned sections.

The debug information from the result after excuting IK_debug.py, given by kyslef, shows the low error rates for the theta values and the solving time under 15 seconds.

![IK_debug][image1]

The traces of the end effector seem to be very off sometimes and slows down the entire process. In addtion, this random plan disruption cannot be prevented as the inverse kinematics should follow the trajectories, even if these are not optimised or realistic.
The planner provides some erroneous plans and the robot dances for a while.

![Asurd plans][image2]

The image below shows the completion of the objective; pick and drop 10 cans successfully. The average IK solver time was 18 seconds.

![Completion pass][image3]
Regarding to speed, simplify in the loop was a main reason for slow calculation. By taking out one time initilisation variables or classes outside the for loop routine and reducing the heavy processing function calls inside, 15 seconds for each calculation, invariant to the number of trajectory points from the service request, unless more than 150 points.

After commenting out dead code, which are in the forward kinematics calculation, *T4_5* to *T0_EEF*, speed gain was big, reducing the calculation time by 15 seconds; only takes 3-4 seconds to finish calculation. To gain more speed, those matrix preparation code can be placed outside of if else loop but inside the ros spin.

![Optimised operation][image4]
![Optimised operation debug][image5]
Speaking of the programming language, python is a slow language, and is not suitable for realtime operations. The chart below shows python is 10 to 100 times slower compared to c excuting the same code. If this project need to be on production, this test bed with the dependent modules must be migrated to C/C++ approperiately to gain the realtime operability.

![How many times slower][image6]

The sympy library is good for showing the formula creation for a generalised form, once the system, the machine setting, is determined, this can be ported to a low level memory manipulation, array or even pointers. Especially in this form many repeatative trigometry calculation is expected. We only need a few matrix, including *T0_EEF* and *T0_3*, these two matrix can be made to an array and boost the calculation speed instead of using sympy library. 


Excessive rotations of end effector was observed. I assume this is due to the multiple solutions for each theta angles, sinage duality from square root functions and also the nature of revolute joints. The improvement for this issue is investigated conditioning the IK solver to choose more realistic solution, preferring a shorter distance or angle amongst the solutions.


