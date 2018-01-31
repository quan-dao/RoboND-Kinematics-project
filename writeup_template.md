## Project: Kinematics Pick & Place
---

[//]: # (Image References)

[image1]: ./misc_images/kr210_DH_convention.png
[image2]: ./misc_images/misc3.png
[image3]: ./misc_images/misc2.png

## 1. Kinematic Analysis
### 1.1 Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

To perform the Forward kinematics analysis of KR210 manipulator, local frames of robot links need to be established. Using the Denavit-Hartenberg (DH) convention described in [1], a local frame is attached to each robot links as in Fig.1. It is worth to notice that robot joints are indexed from `1` to `n`, while robot links and their associated local frame are index from `0` to `n`. Therefore, joint `i` connects link `i-1` and link `i`. Furthermore, joint `i` is rigidly attached to link `i-1`, so its movement leads to the movement of link `i` and frame `i`.

![alt text][image1]
*Fig.1 The local frame assigned to each of robot links (this figure is adopted from [2])*

Fig.1 has identified the four elements of each row of robot's DH table which are twist angles (`alpha(i-1)`), links length (`a(i-1)`), joints angle (`theta(i)`), and links offset (`d(i)`). While the first two elements respectively define the angle and distance between two adjacent frames' `z` axis (frame `i-1` and frame `i`), the last two defines the equivalence of adjacent frames' `x` axis. It is essential for the accuracy of the kinematics analysis to know the positive direction of these elements. For the elements associated with z axis, the positive direction is determined by x axis, and vice versa. The question is which frame's axis is chosen. The answer is the index of these elements. In the case of `alpha` and `a` which index is `i-1`, the `x(i-1)` axis of frame `i-1` defines their positive direction. On the other hand, the `z(i)` axis of frame `i` determines the positive direction of `theta` and `d`.  

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | L1 | qi
1->2 | - pi/2 | L2 | 0 | -pi/2 + q2
2->3 | 0 | 0 | 0 | 0
3->4 |  0 | 0 | 0 | 0
4->5 | 0 | 0 | 0 | 0
5->6 | 0 | 0 | 0 | 0
6->EE | 0 | 0 | 0 | 0


#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

And here's where you can draw out and show your math for the derivation of your theta angles.

![alt text][image2]

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results.


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  


And just for fun, another example image:
![alt text][image3]
