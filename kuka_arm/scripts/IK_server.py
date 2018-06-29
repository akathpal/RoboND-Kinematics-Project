#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# Author: Abhishek Kathpal

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *
from matplotlib import pyplot as plt
import numpy as np

R0_3 = None
q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')	
# Create Modified DH parameters
s = {alpha0: 0,      a0:   0,    d1: 0.75,  q1: q1,
	 alpha1: -pi/2,  a1: 0.35,   d2: 0,     q2: q2 - pi/2,  
	 alpha2: 0,      a2: 1.25,   d3: 0,     q3: q3,
	 alpha3: -pi/2,  a3: -0.054, d4: 1.50,  q4: q4,
	 alpha4: pi/2,   a4:   0,    d5: 0,     q5: q5,
	 alpha5: -pi/2,  a5:   0,    d6: 0,     q6: q6,
	 alpha6: 0,      a6:   0,    d7: 0.303, q7: 0}	     


# Define Modified DH Transformation matrix
def Tf_mat(alpha,a,d,q):
	Tf = Matrix([[            cos(q),           -sin(q),           0,             a],
				 [ sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
			 	 [ sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
			 	 [                 0,                 0,           0,             1]])
	return Tf

def rotation_wc():
	#Computing R0_3

	# Create individual transformation matrices
	T0_1 = Tf_mat(alpha0,a0,d1,q1).subs(s)
	T1_2 = Tf_mat(alpha1,a1,d2,q2).subs(s)
	T2_3 = Tf_mat(alpha2,a2,d3,q3).subs(s)
	
	R0_3 = T0_1[0:3,0:3] * T1_2[0:3,0:3] * T2_3[0:3,0:3]
	
	return R0_3

def fwd_kinematics(theta):

	# Create individual transformation matrices
	#T0_1 = Tf_mat(alpha0,a0,d1,q1).subs(s)
	#T1_2 = Tf_mat(alpha1,a1,d2,q2).subs(s)
	#T2_3 = Tf_mat(alpha2,a2,d3,q3).subs(s)
	#T3_4 = Tf_mat(alpha3,a3,d4,q4).subs(s)
	#T4_5 = Tf_mat(alpha4,a4,d5,q5).subs(s)
	#T5_6 = Tf_mat(alpha5,a5,d6,q6).subs(s)
	#T6_G = Tf_mat(alpha6,a6,d7,q7).subs(s)
	#T0_G = T0_1*T1_2*T2_3*T3_4*T4_5*T5_6*T6_G

	T0_G = Matrix([[(((sin(q2)*cos(q1)*cos(q3) + sin(q3)*cos(q1)*cos(q2))*cos(q4) + sin(q1)*sin(q4))*cos(q5) + (-sin(q2)*sin(q3)*cos(q1) + cos(q1)*cos(q2)*cos(q3))*sin(q5))*cos(q6) - ((sin(q2)*cos(q1)*cos(q3) + sin(q3)*cos(q1)*cos(q2))*sin(q4) - sin(q1)*cos(q4))*sin(q6), -(((sin(q2)*cos(q1)*cos(q3) + sin(q3)*cos(q1)*cos(q2))*cos(q4) + sin(q1)*sin(q4))*cos(q5) + (-sin(q2)*sin(q3)*cos(q1) + cos(q1)*cos(q2)*cos(q3))*sin(q5))*sin(q6) - ((sin(q2)*cos(q1)*cos(q3) + sin(q3)*cos(q1)*cos(q2))*sin(q4) - sin(q1)*cos(q4))*cos(q6), -((sin(q2)*cos(q1)*cos(q3) + sin(q3)*cos(q1)*cos(q2))*cos(q4) + sin(q1)*sin(q4))*sin(q5) + (-sin(q2)*sin(q3)*cos(q1) + cos(q1)*cos(q2)*cos(q3))*cos(q5), -0.303*((sin(q2)*cos(q1)*cos(q3) + sin(q3)*cos(q1)*cos(q2))*cos(q4) + sin(q1)*sin(q4))*sin(q5) + 0.303*(-sin(q2)*sin(q3)*cos(q1) + cos(q1)*cos(q2)*cos(q3))*cos(q5) - 1.5*sin(q2)*sin(q3)*cos(q1) - 0.054*sin(q2)*cos(q1)*cos(q3) + 1.25*sin(q2)*cos(q1) - 0.054*sin(q3)*cos(q1)*cos(q2) + 1.5*cos(q1)*cos(q2)*cos(q3) + 0.35*cos(q1)], [(((sin(q1)*sin(q2)*cos(q3) + sin(q1)*sin(q3)*cos(q2))*cos(q4) - sin(q4)*cos(q1))*cos(q5) + (-sin(q1)*sin(q2)*sin(q3) + sin(q1)*cos(q2)*cos(q3))*sin(q5))*cos(q6) - ((sin(q1)*sin(q2)*cos(q3) + sin(q1)*sin(q3)*cos(q2))*sin(q4) + cos(q1)*cos(q4))*sin(q6), -(((sin(q1)*sin(q2)*cos(q3) + sin(q1)*sin(q3)*cos(q2))*cos(q4) - sin(q4)*cos(q1))*cos(q5) + (-sin(q1)*sin(q2)*sin(q3) + sin(q1)*cos(q2)*cos(q3))*sin(q5))*sin(q6) - ((sin(q1)*sin(q2)*cos(q3) + sin(q1)*sin(q3)*cos(q2))*sin(q4) + cos(q1)*cos(q4))*cos(q6), -((sin(q1)*sin(q2)*cos(q3) + sin(q1)*sin(q3)*cos(q2))*cos(q4) - sin(q4)*cos(q1))*sin(q5) + (-sin(q1)*sin(q2)*sin(q3) + sin(q1)*cos(q2)*cos(q3))*cos(q5), -0.303*((sin(q1)*sin(q2)*cos(q3) + sin(q1)*sin(q3)*cos(q2))*cos(q4) - sin(q4)*cos(q1))*sin(q5) + 0.303*(-sin(q1)*sin(q2)*sin(q3) + sin(q1)*cos(q2)*cos(q3))*cos(q5) - 1.5*sin(q1)*sin(q2)*sin(q3) - 0.054*sin(q1)*sin(q2)*cos(q3) + 1.25*sin(q1)*sin(q2) - 0.054*sin(q1)*sin(q3)*cos(q2) + 1.5*sin(q1)*cos(q2)*cos(q3) + 0.35*sin(q1)], [-(-sin(q2)*sin(q3) + cos(q2)*cos(q3))*sin(q4)*sin(q6) + ((-sin(q2)*sin(q3) + cos(q2)*cos(q3))*cos(q4)*cos(q5) + (-sin(q2)*cos(q3) - sin(q3)*cos(q2))*sin(q5))*cos(q6), -(-sin(q2)*sin(q3) + cos(q2)*cos(q3))*sin(q4)*cos(q6) - ((-sin(q2)*sin(q3) + cos(q2)*cos(q3))*cos(q4)*cos(q5) + (-sin(q2)*cos(q3) - sin(q3)*cos(q2))*sin(q5))*sin(q6), -(-sin(q2)*sin(q3) + cos(q2)*cos(q3))*sin(q5)*cos(q4) + (-sin(q2)*cos(q3) - sin(q3)*cos(q2))*cos(q5), -0.303*(-sin(q2)*sin(q3) + cos(q2)*cos(q3))*sin(q5)*cos(q4) + 0.303*(-sin(q2)*cos(q3) - sin(q3)*cos(q2))*cos(q5) + 0.054*sin(q2)*sin(q3) - 1.5*sin(q2)*cos(q3) - 1.5*sin(q3)*cos(q2) - 0.054*cos(q2)*cos(q3) + 1.25*cos(q2) + 0.75], [0, 0, 0, 1]])

	FK = T0_G.evalf(subs={q1: theta[0],q2: theta[1],q3: theta[2],q4: theta[3],q5: theta[4],q6: theta[5]})
	your_ee = [FK[0,3],FK[1,3],FK[2,3]]
	return your_ee

def inv_kinematics(pW,R0_3,Rot_G):
	
	theta1 = atan2(pW[1],pW[0]) 

	#Solving theta2 and theta3 angles from the projected triangle using cosine rule
	lenA = 1.501
	lenB = sqrt(pow((sqrt(pW[0]*pW[0]+pW[1]*pW[1])-0.35),2) + pow((pW[2]-0.75),2))
	lenC = 1.25

	A = acos((lenB*lenB + lenC*lenC - lenA*lenA)/(2*lenB*lenC))
	B = acos((lenA*lenA + lenC*lenC - lenB*lenB)/(2*lenA*lenC))
	C = acos((lenA*lenA + lenB*lenB - lenC*lenC)/(2*lenB*lenA))

	theta2 = pi/2 - A -atan2(pW[2]-0.75,sqrt(pW[0]*pW[0]+pW[1]*pW[1])-0.35)   
	theta3 = pi/2 - (B+0.036)


	R0_3 = R0_3.evalf(subs={q1: theta1,q2: theta2, q3: theta3})
	#LU Decomposition is used for finding the inverse of R0_3
	R3_6 = R0_3.inv("LU")*Rot_G
	#print(R3_6)
	#deriving euler angles from composition of rotation matrix
	theta4 = atan2(R3_6[2,2], -R3_6[0,2])

	theta5 = atan2(sqrt(R3_6[0,2]*R3_6[0,2] + R3_6[2,2]*R3_6[2,2]),R3_6[1,2])

	theta6 = atan2(-R3_6[1,1], R3_6[1,0])
	theta = [theta1, theta2, theta3, theta4, theta5, theta6]
	return theta

def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

		#T0_G = T0_1*T1_2*T2_3*T3_4*T4_5*T5_6*T6_G
		
		# Initialize service response
		joint_trajectory_list = []
		ee_offset = []
		ee_offset.append(0)

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
			 
			r,p,y = symbols('r p y')
			Rot_x = Matrix([[      1,      0,       0],
							[      0, cos(r), -sin(r)],
				   			[      0, sin(r),  cos(r)]])
			Rot_y = Matrix([[ cos(p),      0,  sin(p)],
							[      0,      1,       0],
							[-sin(p),      0,  cos(p)]])
			Rot_z = Matrix([[ cos(y),-sin(y),       0],
							[ sin(y), cos(y),       0],
							[      0,      0,       1]])         

			#Orientation correction of gripper link as defined in URDF vs DH parameters
			Rot_G = Rot_z*Rot_y*Rot_x
			Rot_corr = Rot_z.subs(y,radians(180))*Rot_y.subs(p,radians(-90))
			Rot_G = Rot_G*Rot_corr 
			Rot_G = Rot_G.subs({'r':roll,'p':pitch,'y':yaw})

			#End-effector position
			pG = Matrix([[px],
						 [py],
						 [pz]])

			#Wrist Center Position Computation
			pW = pG - (0.303)*Rot_G[:,2]

			#Inverse Kinematics Output Joint Angles
			theta = inv_kinematics(pW,R0_3,Rot_G)

			# Error Computing between calculated pose from forward kinematics and input end-effctor poses
			# your_ee = fwd_kinematics(theta)
			# ee_x_e = abs(your_ee[0]-px)
			# ee_y_e = abs(your_ee[1]-py)
			# ee_z_e = abs(your_ee[2]-pz)
			# ee_offset.append(sqrt(ee_x_e**2 + ee_y_e**2 + ee_z_e**2))
			
			# #Printing Error
			# print ("\nEnd effector error for x position is: %04.8f" % ee_x_e)
			# print ("End effector error for y position is: %04.8f" % ee_y_e)
			# print ("End effector error for z position is: %04.8f" % ee_z_e)
			# print ("Overall end effector offset is: %04.8f units \n" % ee_offset[x])

			# #Plotting Error
			# plt.plot([x,x+1],[ee_offset[x],ee_offset[x+1]],'or-',linewidth=2)
			# plt.draw()
			# plt.pause(0.5)
			
			
			joint_trajectory_point.positions = np.float64(theta)
			joint_trajectory_list.append(joint_trajectory_point)

		rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
		#plt.show()
		#plt.pause(1)
		# plt.close()
		
		return CalculateIKResponse(joint_trajectory_list)


def IK_server():
	global R0_3
	# initialize node and declare calculate_ik service
	rospy.init_node('IK_server')
	
	#R0_3 can be calculated using rotation_wc function, but to reduce running time computed R0_3 in form of theta can be used
	#R0_3 = rotation_wc() 
	R0_3 = Matrix([[sin(q2)*cos(q1)*cos(q3) + sin(q3)*cos(q1)*cos(q2), -sin(q2)*sin(q3)*cos(q1) + cos(q1)*cos(q2)*cos(q3), -sin(q1)], [sin(q1)*sin(q2)*cos(q3) + sin(q1)*sin(q3)*cos(q2), -sin(q1)*sin(q2)*sin(q3) + sin(q1)*cos(q2)*cos(q3), cos(q1)], [-sin(q2)*sin(q3) + cos(q2)*cos(q3), -sin(q2)*cos(q3) - sin(q3)*cos(q2), 0]])
	
	s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
	
	print "Ready to receive an IK request"
	rospy.spin()

if __name__ == "__main__":
    IK_server()

 
