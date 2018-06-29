## Udacity RoboND Project: Kinematics Pick & Place
---

[//]: # (Image References)
[image1]: ./misc_images/misc1.png
[image2]: ./misc_images/misc2.png
[image3]: ./misc_images/0.jpeg
[image4]: ./misc_images/1.jpeg
[image5]: ./misc_images/2.jpeg
[image6]: ./misc_images/misc3.png
[image7]: ./misc_images/3.jpeg
[image8]: ./misc_images/4.png
# Writeup 

![alt text][image2]
#### Here I will consider the [rubric points](https://review.udacity.com/#!/rubrics/972/view) individually and describe how I addressed each point in my implementation.  

---

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

![alt text][image1]
I used the kr210 urdf file to find the values of link length parameters as mentioned the lecture videos. The modified DH Parameters are shown in the table below:

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | q1
1->2 | - pi/2 | 0.35 | 0 | -pi/2 + q2
2->3 | 0 | 1.25 | 0 | q3
3->4 |  -pi/2 | -0.054 | 1.50 | q4
4->5 | pi/2 | 0 | 0 | q5
5->6 | -pi/2 | 0 | 0 | q6
6->EE | 0 | 0 | 0.303 | 0

![alt text][image3]

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

After deriving the modified DH parameters, I computed the individual transformation matrices from one link to another. Then, by multiplying the individual transformation matrices I got the final transformation matrix between base_link and end_effector link. 


T0_1 = [[             cos(q1),            -sin(q1),            0,              a0],
            [ sin(q1)*cos(alpha0), cos(q1)*cos(alpha0), -sin(alpha0), -sin(alpha0)*d1],
            [ sin(q1)*sin(alpha0), cos(q1)*sin(alpha0),  cos(alpha0),  cos(alpha0)*d1],
                  [                   0,                   0,            0,               1]],
 All the other transformation matrices can be written in similar way.
 The final homogeneous teanformation matrix can be computed by:
 T0_G = T0_1 x T1_2 x T2_3 x T3_4 x T4_5 x T5_6 x T6_G
    
I have used these equations in the forward-inverse kinematics notebook file which is inside the Kinematics-Notebook folder. This notebook was my test for solving the ik problem without using ros. 

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

Derivation of inverse Kinematics equations is shown below: 

![alt text][image4]



Using the wrist center the theta can be compued with atan2 function.
![alt text][image5]

```python
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
```
Theta 2 and Theta3 can be computed using cosine rule for a triangle shown in figure below. Once we have first three angles, R0_3 can be computed and later using R0_3, we can find the vale of R3_6.
![alt text][image6]

From R3_6, we can extract the Euler angles from the equations shown in image below:
![alt text][image7]



### Project Implementation

Initial steps:
Give permission to all the files in scripts folder by using chmod u+x or 755 ...filename.Follow the 2 steps to run the code:
1.Run safe_spawner.sh 
2.Run python Ik_server.py to run the inverse kinematics solver file.
3.Press Next in Rviz

I have attached the image in the above section which explains how i get the inverse kinematics equations. 
I have also verified the end-effector poses i received as input by comparing them with the poses that i got from forward kinematics using computed thetas. This is done in Ik_debug.py file.

I also plotted the error graph in my Ik_server implementation. The code is well commented. All the steps are explained there. 

I made video of the whole pickup and place project. FileName - PickPlace_Simulation

I also attached the plotted error in the image below:
![alt text][image8]

I have commented the plotting code (line 165-181 and 189) in Ik_server file because sometimes i am getting segmentation fault because of plotting the error.

But without plotting error,  my implementation is able to complete 8/10 cycles.



