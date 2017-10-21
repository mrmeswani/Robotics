## Project: Kinematics Pick & Place

---




[//]: # (Image References)

[image1]: ./FK.png
[image2]: ./pic1.png
[p1]: ./solutions/p1.jpg
[p2]: ./solutions/p2.jpg
[p3]: ./solutions/p3.jpg
[p4]: ./solutions/p4.jpg
[p5]: ./solutions/p5.jpg
[p6]: ./solutions/p6.jpg
[p7]: ./solutions/p7.jpg
[p8]: ./solutions/p8.jpg
[p9]: ./solutions/p9.jpg


## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
## Note the code is included in ./kuka\_arm/scripts IK_server.py


---
### Writeup / README



### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

The Kuka ARM schematics is as shown below:
![alt text][image1]

THE URDF file gives the positions relative to its parent and hence that must be taken into account, We choose the value of d1 to be 0.75 so that x1 intersects the next joint. The offset for d2 and d3 are zero since their X-axes are coincident, similar for d4 and d5. The DH parameter table is shown below:

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | q1
1->2 | - pi/2 | 0.35 | 0 | -pi/2 + q2
2->3 | 0 | 1.25 | 0 | q3
3->4 |  -pi/2 | -0.054 | 1.50 | q4
4->5 | pi/2 | 0 | 0 | q5
5->6 | -pi/2 | 0 | 0 | q6
6->EE | 0 | 0 | 0.303 | q7



#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.
### Transformation matrixes based on DH params
	def TF_Matrix(alpha, a, d, q):
		TF = Matrix([	[cos(q), -sin(q), 0, a],
				[sin(q)*cos(alpha), cos(q)*cos(alpha),-sin(alpha), -sin(alpha)*d],
				[sin(q)*sin(alpha), cos(q)*sin(alpha), cos(alpha), cos(alpha)*d],
				[0,0,0,1]])
		return TF
	# Create individual transformation matrices
	#
	#
	T0_1 = TF_Matrix(alpha0, a0, d1, q1).subs(DH_Table)	
	T1_2 = TF_Matrix(alpha1, a1, d2, q2).subs(DH_Table)	
	T2_3 = TF_Matrix(alpha2, a2, d3, q3).subs(DH_Table)	
	T3_4 = TF_Matrix(alpha3, a3, d4, q4).subs(DH_Table)	
	T4_5 = TF_Matrix(alpha4, a4, d5, q5).subs(DH_Table)	
	T5_6 = TF_Matrix(alpha5, a5, d6, q6).subs(DH_Table)	
	T6_EE =TF_Matrix(alpha6, a6, d7, q7).subs(DH_Table)

	T0_EE = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_EE	



#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.


### Theta angles

![alt text][image2]  

For a spherical wrist, the first 3 joints determine the wrist position. The last three joint determines its orientation. 

Once we have the Rotation matrix from the roll pitch and yaw we can derive the wrist center using rotation matrix and end effector poses given by the planner. 
We can then derive theta1 from wrist center. 

The first theta angle is simple derived be reflecting the wrist center to the x-y plane and then deriving the angle as shown. 
![alt text][p1]  


To calculate theta2 the triangle generate by joints 2, 3 and wrist center. For angle theta3 we need to consider the triangle joint 2,3,5 wrt wrist center as there is a subtle dip from joint 3 to joint 4 which we need to account for, the figures below show the equations we can derive to solve for theta2 and theta 3.
![alt text][p2] 
![alt text][p3] 
![alt text][p4] 


Once we have the theta angles we can derive the matrix R0_3 for the first three joints. 
Since the total transform matrix cab be expressed as R0_3\*R3_6 = RotRPY, we can get the numeric solution to R3_6 = Inverse(R0_3)\*RotRPY. 
I noticed that in my code that I can use the computationally less expensive transpose since R0_3 is a square matrix to make things go a bit faster.

We can get the symbolic form of R3_6_symb = T3_4\*T4_5\*T5_6. 
Now we can solve R3_6_symb = R3_6 for angles theta4-6 which is shown below.
![alt text][p5]
![alt text][p6] 
![alt text][p7] 
![alt text][p8] 
![alt text][p9] 



The final angles as derived above are:  	
theta4 = atan2(R3\_6[2,2], -R3\_6[0,2])  
theta5 = atan2(sqrt(R3\_6[0,2] * R3\_6[0,2] + R3\_6[2,2] * R3\_6[2,2]), R3\_6[1,2])  
theta6 = atan2(-R3\_6[1,1], R3\_6[1,0])

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


For the basic implementation I do see the solutions taking a bit longer which maybe due to the laptop I am running having limited resources and takes a few minutes to get the complete solution. I did calculate the Forward kinematics of my transformations and it appears to be within a reasonable error range. I did not howerver plot them and if given time I would both optimize to reduce the error and also try to make my code faster.

Attached is a video of a pick and place operation. As you can see in the video my error is not zero, but still the robot kuka arm is successfully placing objects in the bin. Overall I observe  that it follows the plan albeit with some deviation. Since the matrix is square I sped up the code to to use transpose instead of the more computationally expensive LU decomposition.

[![IMAGE ALT TEXT HERE](http://img.youtube.com/vi/kuUOJvRvXFk/0.jpg)](https://youtu.be/kuUOJvRvXFk)


