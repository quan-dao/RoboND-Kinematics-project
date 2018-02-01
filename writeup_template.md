## Project: Kinematics Pick & Place
---

[//]: # (Image References)

[image1]: ./misc_images/kr210_DH_convention.png
[image2]: ./misc_images/kr210_urdf.PNG
[image3]: ./misc_images/misc2.png

## 1. Forward Kinematic
### 1.1 Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

To perform the Forward kinematics analysis of KR210 manipulator, local frames of robot links need to be established. Using the Denavit-Hartenberg (DH) convention described in [1], a local frame is attached to each of robot links as in Fig.1. It is worth to notice that  
* Joints are indexed from `1` to `n`, while robot links and their associated local frame are index from `0` to `n`. Therefore, joint `i` connects link `i-1` and link `i`.
* Joint `i` is rigidly attached to link `i-1`, so its movement leads to the movement of link `i` and frame `i`.

![alt text][image1]
*Fig.1 The local frame established by DH convention (this figure is adopted from [2])*

Fig.1 has identified the four parameters of each row of robot's DH table which are twist angles (`alpha(i-1)`), links length (`a(i-1)`), joints angle (`theta(i)`), and links offset (`d(i)`). While the first two parameters respectively define the angle and distance between two adjacent frames' `z` axis (frame `i-1` and frame `i`), the last two defines the equivalence of adjacent frames' `x` axis. It is essential for the accuracy of the kinematics analysis to know the positive direction of these parameters. For the parameters associated with z axis, the positive direction is determined by x axis, and vice versa. The question is which frame's axis is chosen. The answer is the index of these parameters. In the case of `alpha` and `a` which index is `i-1`, the `x(i-1)` axis of frame `i-1` defines their positive direction. On the other hand, the `z(i)` axis of frame `i` determines the positive direction of `theta` and `d`.  

The arrangement of local frames origin position and axis in Fig.1 makes a lot of DH table's parameters equal to zero. The nonzero parameters are annotated by letters with index in Fig.1. These nonzero parameters' value are defined by the URDF file. Having the same idea of DH convention, this file describes our robot by establishing local frames which represents links position and orientation. However, there are two features that differentiate the convention of `URDF` to that of `DH`.
* The origin of frame `i` is always placed at joint `i` (can be anywhere in DH convention)
* The orientation of frame `i` (i.e. the direction of its axis) can be defined freely (must followed a set of rules in DH convention)

Taking these differences into account, the robot kinematic chain constructed using the information stored in the `kr210.urdf.xacro` file (the joints block) is displayed in Fig.2.

![alt text][image2]
*Fig.2 Robot's kinematic chain established by URDF file convention*

As can be seen in Fig.2, the origin of frame `i` denoted by a black triangle is coincident with joint `i`, and the orientation of every frame is the same as the base frame (frame `0`). Furthermore, matching the dimensions in Fig.2 with the nonzero parameters of DH table in Fig.1 yields these parameters' value. Hence, DH table is derived as following,          

i | alpha(i-1) | a(i-1) | d(i) | theta(i)
--- | --- | --- | --- | ---
1 | 0 | 0 | 0.75 | q1
2 | - pi/2 | 0.35 | 0 | q2 - pi/2
3 | 0 | 1.25 | 0 | q3
4 |  -pi/2 | -0.054 | 1.5 | q4
5 | pi/2 | 0 | 0 | q5
6 | -pi/2 | 0 | 0 | q6
7 | 0 | 0 | 0.303 | 0

It is important to stress that row `i` of the DH table represents the pose of frame `i` relative to frame `i - 1`. The gripper frame is indexed by 7, so the last row of this table represents of pose the gripper relative to frame 6.

### 1.2. Using the DH parameter table, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link.

The homogeneous transformation which represents the pose of frame `i` relative to frame `i - 1` established using the DH parameters is

<img src="http://latex.codecogs.com/svg.latex?_{i}^{i-1}\textrm{T} = R_X(\alpha_{i-1})D_X(a_{i-1})R_Z(\theta_i)D_Z(d_i)" border="0"/>

Expanding the equation above yields

<img src="http://latex.codecogs.com/svg.latex?_{i}^{i-1}\textrm{T} = \begin{bmatrix}
c\theta_i &-s\theta_i  &0  &a_{i-1} \\
 s\theta_ic\alpha_{i-1} &c\theta_ic\alpha_{i-1}  &-s\alpha_{i-1}  &-s\alpha_{i-1}d_i \\
 s\theta_is\alpha_{i-1} &c\theta_is\alpha_{i-1}  &c\alpha_{i-1}  &c\alpha_{i-1}d_i \\
 0 &0  &0  &1
\end{bmatrix}" border="0"/>

The global pose (relative to base frame) of the gripper frame is obtained by consecutively perform `body` transform on frame 0 until it is coincident with the gripper frame. Therefore, the global pose of the gripper frame is

<img src="http://latex.codecogs.com/svg.latex?_{G}^{0}\textrm{T} = _{1}^{0}\textrm{T}\:_{2}^{1}\textrm{T}\: \cdots\: _{6}^{5}\textrm{T}\:_{G}^{6}\textrm{T}" border="0" />   

## 2. Inverse Kinematic
### 2.1 Wrist Center Position
The Wrist Center (WC) is where the last three joints' axis intersect. As shown in Fig.1, Wrist Center can be either `O4, O5`, or `O6`. To minimize the number of matrix involved in the process of deriving the WC position, `O4` is chosen.

And here's where you can draw out and show your math for the derivation of your theta angles.


### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results.


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  


And just for fun, another example image:
![alt text][image3]
