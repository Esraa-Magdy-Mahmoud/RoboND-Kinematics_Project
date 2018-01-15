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
import pickle

def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
		
        ### Your FK code here
        # Create symbols
        q1 , q2 ,q3 = symbols('q1 q2 q3')  #roll,pitch,yaw
        ## Create symbols for joint variables 
        theta_1 , theta_2 , theta_3 , theta_4 , theta_5 , theta_6 , theta_7 = symbols('theta_1:8') 
        alpha_0 , alpha_1 , alpha_2 , alpha_3 , alpha_4 , alpha_5 , alpha_6 = symbols('alpha0:7') 

        d1 , d2 , d3 , d4 , d5 , d6 , d7 = symbols('d1:8') 
        a0 , a1 , a2 , a3 , a4 , a5 , a6 = symbols('a0:7')
    #######All the following Calculations are pre calculated and saved into the files loaded#####
        s = pickle.load( open( "DH_Params.p", "rb"))             
        T0_g = pickle.load( open( "Total_Transform.p", "rb" ))
        R_corr = pickle.load( open( "R_corr.p", "rb"))
        Rrpy = pickle.load( open( "rpy.p", "rb" ))
        R0_3 = pickle.load( open( "R0_3.p", "rb" ))     
        ###

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
            EE_pose = Matrix([[px],
                             [py],
                             [pz]])

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])
            ### Your IK code here 
	    # Compensate for rotation discrepancy between DH parameters and Gazebo
            
            
            Rrpy = Rrpy.evalf(subs={q1:roll ,q2:pitch ,q3:yaw })
            WC = EE_pose - (0.303) * Rrpy[:,2] #0.303 = d7
	    #
	    # Calculate joint angles using Geometric IK method
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
            #Substituting then multiplying to make it faster
            #R0_1 = T0_1[0:3,0:3].subs(theta_1, theta1)
            #R1_2 = T1_2[0:3,0:3].subs(theta_2, theta2)
            #R2_3 = T2_3[0:3,0:3].subs(theta_3, theta3)
            #R0_3 = R0_1 * R1_2  *R2_3 
            R0_3 = R0_3.evalf(subs={theta_1: theta1, theta_2:theta2 , theta_3:theta3})
            #R3_6 = R0_3.inv("LU") * Rrpy
    
            # for a rotation matrix the transpose of the matrix equals to the inverse and it's computationally faster
            R3_6 = Transpose(R0_3) * Rrpy
            
            theta5 = atan2(sqrt((R3_6[0,2]*R3_6[0,2]) + (R3_6[2,2]*R3_6[2,2]))  , R3_6[1,2])  
            if sin(theta5) < 0:

                theta4 = atan2(-R3_6[2,2] , R3_6[0,2] )
                theta6 = atan2(R3_6[1,1], -R3_6[1,0])
            else:
                theta4 = atan2(R3_6[2,2] , -R3_6[0,2] )
                theta6 = atan2(-R3_6[1,1], R3_6[1,0])
       
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