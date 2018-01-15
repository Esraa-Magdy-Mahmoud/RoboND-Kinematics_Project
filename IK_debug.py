from sympy import *
from time import time
from mpmath import radians
import tf
import pickle
'''
Format of test case is [ [[EE position],[EE orientation as quaternions]],[WC location],[joint angles]]
You can generate additional test cases by setting up your kuka project and running `$ roslaunch kuka_arm forward_kinematics.launch`
From here you can adjust the joint angles to find thetas, use the gripper to extract positions and orientation (in quaternion xyzw) and lastly use link 5
to find the position of the wrist center. These newly generated test cases can be added to the test_cases dictionary.
'''

test_cases = {1:[[[2.16135,-1.42635,1.55109],
                  [0.708611,0.186356,-0.157931,0.661967]],
                  [1.89451,-1.44302,1.69366],
                  [-0.65,0.45,-0.36,0.95,0.79,0.49]],
              2:[[[-0.56754,0.93663,3.0038],
                  [0.62073, 0.48318,0.38759,0.480629]],
                  [-0.638,0.64198,2.9988],
                  [-0.79,-0.11,-2.33,1.94,1.14,-3.68]],
              3:[[[-1.3863,0.02074,0.90986],
                  [0.01735,-0.2179,0.9025,0.371016]],
                  [-1.1669,-0.17989,0.85137],
                  [-2.99,-0.12,0.94,4.06,1.29,-4.12]],
              4:[],
              5:[]}


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
    ####------Added Sections------------------#####
      ## Create symbols for joint variables 
    ################################
    q1 , q2 ,q3 = symbols('q1 q2 q3')  #roll,pitch,yaw
        ## Create symbols for joint variables 
    theta_1 , theta_2 , theta_3 , theta_4 , theta_5 , theta_6 , theta_7 = symbols('theta_1:8') 
    alpha_0 , alpha_1 , alpha_2 , alpha_3 , alpha_4 , alpha_5 , alpha_6 = symbols('alpha0:7') 

    d1 , d2 , d3 , d4 , d5 , d6 , d7 = symbols('d1:8') 
    a0 , a1 , a2 , a3 , a4 , a5 , a6 = symbols('a0:7')
        # Create Modified DH parameters
    s = { alpha_0:     0, a0:      0, d1: 0.75,  theta_1: theta_1      ,
          alpha_1: -pi/2, a1:   0.35, d2:    0,  theta_2: theta_2-pi/2 ,
          alpha_2:     0, a2:   1.25, d3:    0,  theta_3: theta_3      ,
          alpha_3: -pi/2, a3: -0.054, d4: 1.50,  theta_4: theta_4      ,
          alpha_4:  pi/2, a4:      0, d5:    0,  theta_5: theta_5      ,
          alpha_5: -pi/2, a5:      0, d6:    0,  theta_6: theta_6      ,
          alpha_6:     0, a6:      0, d7:0.303,  theta_7: 0            ,
        } 
       
                
    # Define Modified DH Transformation matrix
       ##Transfomation_Matrices from i_1 to i
    def Transfom (theta,d,alpha,a):
        TF = Matrix([[    cos(theta) ,           -sin(theta) ,  0          ,  a            ],
            [  sin(theta)*cos(alpha) , cos(theta)*cos(alpha) , -sin(alpha) , -sin(alpha)*d ], 
            [  sin(theta)*sin(alpha) , cos(theta)*sin(alpha) ,  cos(alpha) ,  cos(alpha)*d ],
            [  0                     , 0                     ,  0          ,  1            ]])
        return TF

            
            
        # Create individual transformation matrices
    T0_1 = Transfom(theta_1 ,d1 ,alpha_0 ,a0).subs(s)  
    T1_2 = Transfom(theta_2 ,d2 ,alpha_1 ,a1).subs(s)
    T2_3 = Transfom(theta_3 ,d3 ,alpha_2 ,a2).subs(s)
    T3_4 = Transfom(theta_4 ,d4 ,alpha_3 ,a3).subs(s)
    T4_5 = Transfom(theta_5 ,d5 ,alpha_4 ,a4).subs(s)
    T5_6 = Transfom(theta_6 ,d6 ,alpha_5 ,a5).subs(s)
    
    T6_g = Transfom(theta_7 ,d7 ,alpha_6 ,a6).subs(s)
    #     ##Composition of transformation 
    T0_g = T0_1 * T1_2 *T2_3 * T3_4 *T4_5 *T5_6*T6_g
       # Extract rotation matrices from the transformation matrices
    R_x = Matrix([[1,        0,         0],
                [0,  cos(q1),   -sin(q1)],
                [0,  sin(q1),   cos(q1)]])
    
    R_y = Matrix([[cos(q2),  0,   sin(q2)],
                 [       0,  1,         0],
                 [-sin(q2),  0,   cos(q2)]])
    
    R_z = Matrix([[cos(q3), -sin(q3), 0],
                 [sin(q3),  cos(q3), 0],
    
                 [0,        0,       1]])
    Rrpy = R_z * R_y * R_x
    R_corr = R_z * R_y
    R_corr = R_corr.evalf(subs={q3:pi,q2:-pi/2}) 
    Rrpy = Rrpy * R_corr

    R0_1 = T0_1[0:3,0:3]
    R1_2 = T1_2[0:3,0:3]
    R2_3 = T2_3[0:3,0:3]
    R0_3 = R0_1 * R1_2  *R2_3  
    
    pickle.dump(s , open("DH_Params.p","wb"))
    pickle.dump(T0_g,open("Total_Transform.p","wb"))
    pickle.dump(R_corr,open("R_corr.p","wb"))
    pickle.dump(Rrpy,open("rpy.p","wb"))
    pickle.dump(R0_3,open("R0_3.p","wb"))
    
    
                      


     #####################################
    px = req.poses[x].position.x
    py = req.poses[x].position.y
    pz = req.poses[x].position.z

    EE_pose = Matrix([[px],
                      [py],
                      [pz]])

    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
         [req.poses[x].orientation.x, req.poses[x].orientation.y,
             req.poses[x].orientation.z, req.poses[x].orientation.w])
      
    
    Rrpy = Rrpy.evalf(subs={q1:roll ,q2:pitch ,q3:yaw })
    WC = EE_pose - (0.303) * Rrpy[:,2] #0.303 = d7

    
    ##Intermediate quantities to solve for the angles
    S1 = sqrt((WC[1] * WC[1])+(WC[0] * WC[0] )) - 0.35 #a1=0.35
    S2 = WC[2] - 0.75 #d1=0.75
    S3 = sqrt((S1*S1)+(S2*S2))
    S4 = 1.501 #sqrt((a3 *a3)+(d4 *d4))
    beta1 = atan2(S2 ,S1 )
    beta2 = acos(((S3*S3)+(1.563)-(S4*S4)) /(2.5*S3)) 
    beta3 = acos(((1.5625)+(S4*S4)-(S3*S3)) /(2.5*S4))
    beta4 = 0.0359#atan2(a3,d4)
    ##--

    theta1 = atan2(WC[1],WC[0]) #(atan2(WC_y, WC_x))
    theta2 = (pi/2)-beta1-beta2
    theta3 = (pi/2)-(beta4+beta3)
   
    
    R0_3 = R0_3.evalf(subs={theta_1: theta1, theta_2:theta2 , theta_3:theta3})
    #R3_6 = R0_3.inv("LU") * Rrpy
    R3_6 = Transpose(R0_3) * Rrpy
    
    theta4 = atan2(R3_6[2,2] , -R3_6[0,2] )
    theta5 = atan2(sqrt((R3_6[0,2]*R3_6[0,2]) + (R3_6[2,2]*R3_6[2,2]))  , R3_6[1,2])
    theta6 = atan2(-R3_6[1,1], R3_6[1,0])

    ## 
    ########################################################################################
    
    ########################################################################################
    ## For additional debugging add your forward kinematics here. Use your previously calculated thetas
    ## as the input and output the position of your end effector as your_ee = [x,y,z]

    ## (OPTIONAL) YOUR CODE HERE!
    fk = T0_g.evalf(subs={theta_1:theta1 , theta_2:theta2 ,theta_3:theta3, theta_4:theta4,theta_5:theta5,theta_6:theta6 })

    ## End your code input for forward kinematics here!
    ########################################################################################

    ## For error analysis please set the following variables of your WC location and EE location in the format of [x,y,z]
    your_wc = [WC[0],WC[1],WC[2]] # <--- Load your calculated WC values in this array
    your_ee = [fk[0,3],fk[1,3],fk[2,3]] # <--- Load your calculated end effector value from your forward kinematics
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
    test_case_number = 2

    test_code(test_cases[test_case_number])
