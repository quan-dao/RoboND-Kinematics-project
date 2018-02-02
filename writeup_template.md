## Project: Kinematics Pick & Place
---

[//]: # (Image References)

[image1]: ./misc_images/kr210_DH_convention.png
[image2]: ./misc_images/kr210_urdf.PNG
[image3]: ./misc_images/wc_position.PNG

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
#### 1.2.1 Global pose of frame `i`
The transformation based on DH parameters which transforms frame `i - 1`to frame `i` is

<img src="https://latex.codecogs.com/gif.latex?_{i}^{i-1}\textrm{T}&space;=&space;R_X(\alpha_{i-1})D_X(a_{i-1})R_Z(\theta_i)D_Z(d_i)"/>

Expanding the equation above yields the homogeneous transformation representing the pose of frame `i` relative to frame `i - 1`

<img src="https://latex.codecogs.com/gif.latex?_{i}^{i-1}\textrm{T}&space;=&space;\begin{bmatrix}&space;c\theta_i&space;&-s\theta_i&space;&0&space;&a_{i-1}&space;\\&space;s\theta_ic\alpha_{i-1}&space;&c\theta_ic\alpha_{i-1}&space;&-s\alpha_{i-1}&space;&-s\alpha_{i-1}d_i&space;\\&space;s\theta_is\alpha_{i-1}&space;&c\theta_is\alpha_{i-1}&space;&c\alpha_{i-1}&space;&c\alpha_{i-1}d_i&space;\\&space;0&space;&0&space;&0&space;&1&space;\end{bmatrix}"/>

Frame `i` is obtained by consecutively perform `body` transform on frame 0 until it is coincident with frame `i`. Therefore, the global pose of frame `i` is

<img src="https://latex.codecogs.com/gif.latex?_{G}^{0}\textrm{T}&space;=&space;_{1}^{0}\textrm{T}\:_{2}^{1}\textrm{T}\:&space;\cdots\:&space;_{6}^{5}\textrm{T}\:_{G}^{6}\textrm{T}"/>   

#### 1.2.2 Global orientation of the gripper frame
The Euler angles (the ordered set: {roll, pitch, yaw}) obtained from ROS' Pose message represents the orientation of the gripper frame defined by the URDF file (the small green frame in Fig.2). Given that the convention of the Euler angles (used in ROS) being is `fixed X-Y-Z`, the pose of URDF defined gripper frame is

<img src="https://latex.codecogs.com/gif.latex?^{0}\textrm{ORI}_{G(urdf)}&space;=&space;Rot_Z(yaw)\:&space;Rot_Y(pitch)\:&space;Rot_X(roll)"/>

For the orientation of the gripper frame to be useful for solving Inverse Kinematic problem, the URDF defined gripper frame needs to be transformed to the gripper frame which is defined by the DH convention (the yellow frame in Fig.2). This transformation is composed by a body rotation of `-pi/2` around y axis followed by another body rotation of `pi` around z axis. Therefore, the orientation of the DH convention defined gripper frame is

<img src="https://latex.codecogs.com/gif.latex?^{0}\textrm{ORI}_{G}&space;=&space;^{0}\textrm{ORI}_{G(urdf)}\:&space;Rot_Y(-\pi/2)\:&space;Rot_Z(\pi)"/>

Substitute the oriention of the URDF defined gripper frame into the equation above

<img src="https://latex.codecogs.com/gif.latex?^{0}\textrm{ORI}_{G}&space;=&space;Rot_Z(yaw)\:&space;Rot_Y(pitch)\:&space;Rot_X(roll)\:&space;Rot_Y(-\pi/2)\:&space;Rot_Z(\pi)"/>  

## 2. Inverse Kinematic
### 2.1 Wrist Center Position
In this section, the base frame is alternatively referred to as the `global frame` for convenient.

The Wrist Center (WC) is where the last three joints' axis intersect. As shown in Fig.1, Wrist Center can be either `O4, O5`, or `O6`. To minimize the number of matrix involved in the process of deriving the WC position, the origin of frame `4`, `O4`, is chosen.

The pose of frame `4` relative to the base frame is calculated using the third equation of section 1.2.

<img src="https://latex.codecogs.com/gif.latex?_{4}^{0}\textrm{T}&space;=&space;_{1}^{0}\textrm{T}\:_{2}^{1}\textrm{T}\:_{3}^{2}\textrm{T}\:_{4}^{3}\textrm{T}"/>

Substitute the value of parameters of the DH table's first four rows into equation above and slice and dice the last column of the result to get the global position of `O4` or the Wrist Center

<img src="https://latex.codecogs.com/gif.latex?^{0}\textrm{WC}&space;=&space;\begin{bmatrix}&space;\(1.25s_2-0.054s_{23}&plus;1.5c_{23}&plus;0.35\)c_1\\&space;\(1.25s_2-0.054s_{23}&plus;1.5c_{23}&plus;0.35\)s_1\\&space;1.25c_2-0.054c_{23}-1.5s_{23}+0.75&space;\end{bmatrix}"/>

<img src="https://latex.codecogs.com/gif.latex?\inlines_2,s_{23},c_{23}"/> are respectively represent <img src="https://latex.codecogs.com/gif.latex?\inline\sin(q_2),\sin(q_2+q_3),\cos(q_2+q_3)"/>.On the orther hand, using the global position of the gripper (the column vector `[px; py; pz]`) and its global orientation, the global position of the WC is defined by

<img src="https://latex.codecogs.com/gif.latex?^{0}\textrm{WC}&space;=&space;\begin{bmatrix}p_x\\p_y\\p_z\end{bmatrix}-0.303\:&space;^{0}\widehat{Z}_G"/>

In the equation above <img src="https://latex.codecogs.com/gif.latex?\inline^{0}\widehat{Z}_G"/> is the global coordinate of the Z axis of the gripper frame. Comparing these two equations, the value of `q1` is

<img src="https://latex.codecogs.com/gif.latex?q_1&space;=&space;atan2\({WC}_y,&space;{WC}_x)"border="1"/>

Once the first joint angle (`q1`) is found, it is then used to produce another equation of `q2` and `q3` from either `WCx` or `WCy`.

<img src="https://latex.codecogs.com/gif.latex?1.25s_2-0.054s_{23}&plus;1.5c_{23}=-0.35&plus;\left\{\begin{matrix}&space;{WC}_y/\sin(q_1),&space;if&space;\sin(q_1)&space;\neq&space;0\\&space;{WC}_x/\cos(q_1),&space;otherwise&space;\end{matrix}\right."/>

For simplicity, let's respectivle assign the right hand side of this equation and <img src="https://latex.codecogs.com/gif.latex?\inlinep_z-0.75"> to <img src="https://latex.codecogs.com/gif.latex?\inline\beta,\alpha"/>.This equation together with the equation of `WCz` form a 2x2 system of trogonometric equation.

<img src="https://latex.codecogs.com/gif.latex?\left\{\begin{matrix}&space;1.25c_2-0.054c_{23}-1.5s_{23}=\alpha\\&space;1.25s_2-0.054s_{23}&plus;1.5c_{23}=\beta&space;\end{matrix}\right."/>

Multiply the second equation by the complex unit *`i`* and add it to the first equation to the following complex equation

<img src="https://latex.codecogs.com/gif.latex?1.25e^{iq_2}&plus;(-0.054&plus;i1.5)e^{i(q_2&plus;q_3)}&space;=&space;\alpha&space;&plus;&space;i\beta"/>

Assign <img src="https://latex.codecogs.com/gif.latex?\inline(-0.054&plus;i1.5)"> to <img src="https://latex.codecogs.com/gif.latex?re^{i\gamma}"> and <img src="https://latex.codecogs.com/gif.latex?\inline(\alpha&plus;i\beta)"> to <img src="https://latex.codecogs.com/gif.latex?\inliner_1e^{i\gamma_1}">. Substitute these the equation above

<img src="https://latex.codecogs.com/gif.latex?1.25e^{iq_2}&plus;re^{i(q_2&plus;q_3&plus;\gamma)}&space;=&space;r_1e^{i\gamma_1}"/>

This equation can be illustrated geometrically by Fig.3.

![alt text][image3]
*Fig.3 WC position problem on the complex plane*  

On the complex plane in Fig.3, the red, green, blue vector respectively represent <img src="https://latex.codecogs.com/gif.latex?\inliner_1e^{i\gamma_1},&space;1.25e^{iq_2},&space;re^{i(q_2&plus;q_3&plus;\gamma)}"/>. It is worth to notice that the length of red (`r1`) and blue vector (`r`) as well as the argument (`gamma`) are already known. Using the law of cosine in a triangle, the cyan angle in Fig.3 is computed as following

<img src="https://latex.codecogs.com/gif.latex?\angle&space;cyan=\arccos\frac{r_{1}^{2}&plus;1.25^2-r^2}{2\cdot&space;r_1\cdot&space;1.25}"/>

The value of `q2` is

<img src="https://latex.codecogs.com/gif.latex?q_2&space;=&space;\gamma_1&space;-&space;\angle&space;cyan"border="1"/>

Using the law of sine in a triangle The angle between red and blue vector is

<img src="https://latex.codecogs.com/gif.latex?\angle&space;(\widehat{red},\widehat{blue})&space;=&space;\frac{1.25\sin\angle&space;cyan}{r}"/>

The value of `q3` is

<img src="https://latex.codecogs.com/gif.latex?q_3&space;=&space;\gamma_1&space;&plus;&space;\angle&space;(\widehat{red},\widehat{blue})-(q_2&plus;\gamma)"border="1"/>   

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results.


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  


And just for fun, another example image:
![alt text][image3]
