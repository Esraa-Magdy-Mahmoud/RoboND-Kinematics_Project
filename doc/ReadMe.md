
# Project: Kinematics Pick & Place
## Kinematics Analysis 


### 1-Forward Kinematics demo
![fk.png](attachment:fk.png)
make the Robot model semi-transparent by assigning alpha to 0.3

### 2-DH Parameter table
Links|alpha(i-1)|a(i-1)|di   |theta_i     
--- | --- | --- | --- | ---
0_1  |0         |0     |0.75 | theta_1    
1_2  |-pi/2     |0.35  |0    |theta_2-pi/2
2_3  |0         | 1.25 |0    |theta_3     
3_4  |-pi/2     |-0.054|1.50 |theta_4     
4_5  |pi/2      |0     |0    |theta_5     
5_6  |-pi/2     |0     |0    |theta_6     
6_G  |0         |0     |0.303|0             

* The Parameters are derived from URDF file and with the help of the figure below showing the DH convention while taking into account that the convention of the URDF file is diffrent from the DH convention.
* Given the DH Params, a transofrmation matrix from i to i-1 can be calculated as follows :
 $\begin{pmatrix}
 cos(theta(i)) & -sin(theta(i)) & 0 & a_i-1 \\
 sin(theta(i))*cos(alpha(i-1)) & cos(theta(i))*cos(alpha(i-1)) &-sin(alpha(i-1))& -sin(alpha(i-1))*d(i) \\ 
 sin(theta(i))*sin(alpha(i-1)) & cos(theta(i))*sin(alpha(i-1)) &  cos(alpha(i-1))& cos(alpha(i-1))*d(i) \\
 0&0&0&1\\
\end{pmatrix} $

Implementing the above generic Transformation in a function,using it the individual Transformations will be as following:
    *    T0_1 = Transfom(theta_1 ,d1 ,alpha_0 ,a0)  
    *    T1_2 = Transfom(theta_2 ,d2 ,alpha_1 ,a1)
    *    T2_3 = Transfom(theta_3 ,d3 ,alpha_2 ,a2)
    *    T3_4 = Transfom(theta_4 ,d4 ,alpha_3 ,a3)
    *    T4_5 = Transfom(theta_5 ,d5 ,alpha_4 ,a4)
    *    T5_6 = Transfom(theta_6 ,d6 ,alpha_5 ,a5)
    *    T6_g = Transfom(theta_7 ,d7 ,alpha_6 ,a6)


* Transformation from the end-effector to the base link will be a composition of transformation
    T0_g = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_g

*  The URDF doesn't folow the same convention of the DH parameters,Hence the frames of the DH model will not be the same as Rviz/Gazebo to Compensate for rotation discrepancy between DH parameters and Gazebo a rotation matrix involoves rotation about z axis with 180 degs then about the y axis with -90 degs is implemented.
The following two images illustrate the DH parameters frame convention versus Rviz frame convention.

1-Rviz Convention
![Rviz_conv.png](attachment:Rviz_conv.png)
2-DH Convention 
![DH_conv.png](attachment:DH_conv.png)



### 3-Decouple IK Problem nto Inverse Position Kinematics and inverse Orientation Kinematics
first, To calculate the Wrist Centre pose with respect to the base_link. a Compositions extrinsic Rotations using Euler angles multiplied by the correction rotaion matrix to correct convention are applied.
The comopsition of the mentioned rotations will result in the following matrix:

Rrpy = R_z * R_y * R_x * R_corr 
then WC_position = EE_pose - d7*R

Second, Calculate the first three joint variables:
theta 1 can be easily calculated by atan2(WC_y, WC_z)
theta 2, theta 3 can be calculated using the following calculations
![IK.jpg](attachment:IK.jpg)

The Calculations of the last three joint angles can be done by extracting the rotation matrices from DH Transformations.
The resultant rotation is:
R0_6 = R0_1*R1_2*R2_3*R3_4*R4_5*R5_6
since we calculated the rotation from the base_link to the end effector then Rrpy = R0_6

Then for the last three joints R3_6 = inv(R0_3) * Rrpy 
**Note : for a rotation matrix the inverse of the rotation is the same as its transpose,but calculating the treanspose is not computationally expensive as the inverse.**

from R3_6 the joint angles are found as follows :

theta5= atan2(sqrt(r13*r13 + r33*r33), r23)

to take care of multipe solutions :

    if sin(theta5) < 0:
        theta4 = atan2(-r33, r13)
        theta6 = atan2(r22, -r21)
    else:
        theta4 = atan2(r33, -r13)
        theta6 = atan2(-r22, r21)


**Notes on Ik_server.py***
I did all the calculations once in IK_debug and saved all the outputs into files, so when running the code only needs to look up into the files not doing any complex matrix multiplications.

the Ik solution while loading the pre-calculations from files can be found in approximately less than 0.2 secs
![out.png](attachment:out.png)


# 4- IK_server.py
the code starts with defining the symbols for the joint variables and the yaw, pitch and roll angles.After that, it looks into the files where the pre-calculations are saved and retrives them. 
The pre-calculations involves the DH Parameters dictionary and all the matrices needed in symbolic format, including the total transformation from the base link to the end-effector, the correction rotation matrix, the yaw-pitch-roll rotation matrix and the rotation matrix from base link to the end-effector.
Then , it extracts the position and orientation of the end-effector and calculate the wrist centre and joints angles as mentioned in the previous section.

