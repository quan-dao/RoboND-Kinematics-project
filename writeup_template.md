## Project: Kinematics Pick & Place
---

[//]: # (Image References)

[image1]: ./misc_images/kr210_DH_convention.png
[image2]: ./misc_images/kr210_urdf.PNG
[image3]: ./misc_images/wc_position.PNG
[image4]: ./misc_images/reach_target.PNG
[image5]: ./misc_images/retreive_target.PNG
[image6]: ./misc_images/reach_drop_off.PNG
[image7]: ./misc_images/release_target.PNG

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

<img src="https://latex.codecogs.com/gif.latex?_{i}^{i-1}\textrm{T}&space;=&space;Rot_X(\alpha_{i-1})D_X(a_{i-1})Rot_Z(\theta_i)D_Z(d_i)"/>

Expanding the equation above yields the homogeneous transformation representing the pose of frame `i` relative to frame `i - 1`

<img src="https://latex.codecogs.com/gif.latex?_{i}^{i-1}\textrm{T}&space;=&space;\begin{bmatrix}&space;c\theta_i&space;&-s\theta_i&space;&0&space;&a_{i-1}&space;\\&space;s\theta_ic\alpha_{i-1}&space;&c\theta_ic\alpha_{i-1}&space;&-s\alpha_{i-1}&space;&-s\alpha_{i-1}d_i&space;\\&space;s\theta_is\alpha_{i-1}&space;&c\theta_is\alpha_{i-1}&space;&c\alpha_{i-1}&space;&c\alpha_{i-1}d_i&space;\\&space;0&space;&0&space;&0&space;&1&space;\end{bmatrix}"/>

Respectively substituting each row of DH table to the equation above results in the individual homogeneous transformation matrix as following.

<img src="https://latex.codecogs.com/gif.latex?\inline&space;\begin{matrix}&space;_{1}^{0}\textrm{T}&space;=&space;\begin{bmatrix}&space;\cos(q_1)&space;&&space;-\sin(q_1)&space;&&space;0&space;&&space;0&space;\\&space;\sin(q_1)&space;&&space;\cos(q_1)&space;&&space;0&space;&&space;0&space;\\&space;0&space;&&space;0&space;&&space;1&space;&&space;0.75&space;\\&space;0&space;&&space;0&space;&&space;0&space;&&space;1&space;\end{bmatrix}&space;&&space;_{2}^{1}\textrm{T}&space;=&space;\begin{bmatrix}&space;\sin(q_2)&space;&&space;\cos(q_2)&space;&&space;0&space;&&space;0.35&space;\\&space;0&space;&&space;0&space;&&space;1&space;&&space;0&space;\\&space;\cos(q_2)&space;&&space;-\sin(q_2)&space;&&space;0&space;&&space;0&space;\\&space;0&space;&&space;0&space;&&space;0&space;&&space;1&space;\end{bmatrix}&space;\end{matrix}"/>

<img src="https://latex.codecogs.com/gif.latex?\inline&space;\begin{matrix}&space;_{3}^{2}\textrm{T}&space;=&space;\begin{bmatrix}&space;\cos(q_3)&space;&&space;-\sin(q_3)&space;&&space;0&space;&&space;1.25&space;\\&space;\sin(q_3)&space;&&space;\cos(q_3)&space;&&space;0&space;&&space;0&space;\\&space;0&space;&&space;0&space;&&space;1&space;&&space;0&space;\\&space;0&space;&&space;0&space;&&space;0&space;&&space;1&space;\end{bmatrix}&space;&&space;_{4}^{3}\textrm{T}&space;=&space;\begin{bmatrix}&space;\cos(q_4)&space;&&space;-\sin(q_4)&space;&&space;0&space;&&space;-0.054&space;\\&space;0&space;&&space;0&space;&&space;1&space;&&space;1.5&space;\\&space;-\sin(q_4)&space;&&space;-\cos(q_4)&space;&&space;0&space;&&space;0&space;\\&space;0&space;&&space;0&space;&&space;0&space;&&space;1&space;\end{bmatrix}&space;\end{matrix}"/>

<img src="https://latex.codecogs.com/gif.latex?\inline&space;\begin{matrix}&space;_{5}^{4}\textrm{T}&space;=&space;\begin{bmatrix}&space;\cos(q_5)&space;&&space;-\sin(q_5)&space;&&space;0&space;&&space;0&space;\\&space;0&space;&&space;0&space;&&space;-1&space;&&space;0&space;\\&space;\sin(q_5)&space;&&space;\cos(q_5)&space;&&space;0&space;&&space;0&space;\\&space;0&space;&&space;0&space;&&space;0&space;&&space;1&space;\end{bmatrix}&space;&&space;_{6}^{5}\textrm{T}&space;=&space;\begin{bmatrix}&space;\cos(q_6)&space;&&space;-\sin(q_6)&space;&&space;0&space;&&space;0&space;\\&space;0&space;&&space;0&space;&&space;1&space;&&space;0&space;\\&space;-\sin(q_6)&space;&&space;-\cos(q_6)&space;&&space;0&space;&&space;0&space;\\&space;0&space;&&space;0&space;&&space;0&space;&&space;1&space;\end{bmatrix}&space;\end{matrix}"/>

<img src="https://latex.codecogs.com/gif.latex?\inline&space;_{G}^{6}\textrm{T}&space;=&space;\begin{bmatrix}&space;1&space;&&space;0&space;&&space;0&space;&&space;0&space;\\&space;0&space;&&space;1&space;&&space;0&space;&&space;0&space;\\&space;0&space;&&space;0&space;&&space;1&space;&&space;0.303&space;\\&space;0&space;&&space;0&space;&&space;0&space;&&space;1\end{bmatrix}"/>

Frame `i` is obtained by consecutively perform `body` transform on frame 0 until it is coincident with frame `i`. Therefore, the global pose of frame `i` is

<img src="https://latex.codecogs.com/gif.latex?_{G}^{0}\textrm{T}&space;=&space;_{1}^{0}\textrm{T}\:_{2}^{1}\textrm{T}\:&space;\cdots\:&space;_{6}^{5}\textrm{T}\:_{G}^{6}\textrm{T}"/>   

#### 1.2.2 Global orientation of the gripper frame
The Euler angles (the ordered set: {roll, pitch, yaw}) obtained from ROS' Pose message represents the orientation of the gripper frame defined by the URDF file (the small green frame in Fig.2). Given that the convention of the Euler angles (used in ROS) being is `fixed X-Y-Z`, the orientation of URDF defined gripper frame is

<img src="https://latex.codecogs.com/gif.latex?^{0}\textrm{R}_{G(urdf)}&space;=&space;Rot_Z(yaw)\:&space;Rot_Y(pitch)\:&space;Rot_X(roll)"/>

For the orientation of the gripper frame to be useful in solving Inverse Kinematic problem, the URDF defined gripper frame needs to be transformed to the gripper frame which is defined by the DH convention (the yellow frame in Fig.2). This transformation is composed by a `body` rotation of `-pi/2` around y axis followed by another body rotation of `pi` around z axis. Therefore, the orientation of the DH convention defined gripper frame is

<img src="https://latex.codecogs.com/gif.latex?^{0}\textrm{R}_{G}&space;=&space;^{0}\textrm{R}_{G(urdf)}\:&space;Rot_Y(-\pi/2)\:&space;Rot_Z(\pi)"/>

Substitute the oriention of the URDF defined gripper frame into the equation above

<table><tr><td>
<img src="https://latex.codecogs.com/gif.latex?^{0}\textrm{R}_{G}&space;=&space;Rot_Z(yaw)\:&space;Rot_Y(pitch)\:&space;Rot_X(roll)\:&space;Rot_Y(-\pi/2)\:&space;Rot_Z(\pi)"/>
</td></tr></table>

This equation is expanded into

<img src="https://latex.codecogs.com/gif.latex?\inline&space;_{G}^{0}\textrm{R}&space;=&space;\begin{bmatrix}&space;\sin(p)\cos(r)\cos(y)&plus;\sin(r)\sin(y)&space;&&space;-\sin(p)\sin(r)\cos(y)&plus;\sin(y)\cos(r)&space;&&space;\cos(p)\cos(y)&space;\\&space;\sin(p)\sin(y)\cos(r)-\sin(r)\cos(y)&space;&&space;-\sin(p)\sin(r)\sin(y)-\cos(r)\cos(y)&space;&&space;\sin(y)\cos(p)&space;\\&space;\cos(p)\cos(r)&space;&&space;-\sin(r)\cos(p)&space;&-\sin(p)&space;\end{bmatrix}"/>

Combine this representation of gripper frame's global orientation with its global position `[px;py;pz]` to get the gripper frame's global pose

<img src="https://latex.codecogs.com/gif.latex?\inline&space;_{G}^{0}\textrm{T}&space;=&space;\begin{bmatrix}&space;_{G}^{0}\textrm{R}&space;&&space;\begin{matrix}&space;p_x\\&space;p_y\\&space;p_z&space;\end{matrix}&space;\\&space;\begin{matrix}&space;0&space;&0&space;&0&space;\end{matrix}&space;&1&space;\end{bmatrix}"/>

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

<table><tr><td>
<img src="https://latex.codecogs.com/gif.latex?q_1&space;=&space;atan2\({WC}_y,&space;{WC}_x)"/>
</td></tr></table>

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

<table><tr><td>
<img src="https://latex.codecogs.com/gif.latex?q_2&space;=&space;\gamma_1&space;-&space;\angle&space;cyan"/>
</td></tr></table>

Using the law of sine in a triangle The angle between red and blue vector is

<img src="https://latex.codecogs.com/gif.latex?\angle&space;(\widehat{red},\widehat{blue})&space;=&space;\frac{1.25\sin\angle&space;cyan}{r}"/>

The value of `q3` is

<table><tr><td>
<img src="https://latex.codecogs.com/gif.latex?q_3&space;=&space;\gamma_1&space;&plus;&space;\angle&space;(\widehat{red},\widehat{blue})-(q_2&plus;\gamma)"/>
</td></tr></table>
</td></tr></table>

### 2.2 Gripper orientation

Using the angular position of the first three joints (`q1, q2, q3`) obtained in the previous subsection, the other three joint angles are now calculated so that the DH convention defined gripper frame achieves the orientation which is derived from the Euler angles of the URDF defined gripper frame measured by ROS' Pose message as in subsection 1.2.2.

Because the value of `q1, q2, q3` have been already known, the global pose of frame `3` has been completely defined. This enable the calculation of `q4, q5, q6` through the orientation of the DH convention defined gripper frame relative to frame `3`. The reason frame `3` is chosen to define the orientation of gripper frame instead of frame `0` is that it requires less matrix multiplication (4 in case of frame `3` compared to 7 in case of frame `0`). The pose of the gripper frame relative to frame `3` symbolically calculated with DH parameters is

<img src="https://latex.codecogs.com/gif.latex?_{G}^{3}\textrm{T}&space;=_{4}^{3}\textrm{T}\:&space;_{5}^{4}\textrm{T}\:&space;_{6}^{5}\textrm{T}\:&space;_{G}^{6}\textrm{T}" />

This equation is expanded to

<img src="https://latex.codecogs.com/gif.latex?_{G}^{3}\textrm{T}&space;=&space;\begin{bmatrix}&space;-s_4s_6&plus;c_4c_5c_6&space;&&space;-s_4c_6-s_6c_4c_5&space;&-s_5c_4&space;&-0.303s_5c_4-0.054&space;\\&space;s_5c_6&&space;-s_5s_6&space;&&space;c_5&space;&&space;0.303c_5&plus;1.5&space;\\&space;-s_4c_5c_6-s_6c_4&&space;s_4s_6c_5-c_4c_6&space;&&space;s_4s_5&space;&&space;0.303s_4s_5&space;\\&space;0&&space;0&&space;0&&space;1&space;\end{bmatrix}"/>

Assign the first three lines and three rows of the matrix above to <img src="https://latex.codecogs.com/gif.latex?\inline_{G}^{3}\textrm{R}"/>. The numerical value of<img src="https://latex.codecogs.com/gif.latex?\inline_{G}^{3}\textrm{R}"/> is

<img src="https://latex.codecogs.com/gif.latex?_{G}^{3}\textrm{R}&space;=&space;_{0}^{3}\textrm{R}\:&space;_{G}^{0}\textrm{R}&space;=&space;_{3}^{0}\textrm{R}^T\:&space;_{G}^{0}\textrm{R}"/>

Note that the right hand side of this above is fully known. <img src="https://latex.codecogs.com/gif.latex?\inline_{3}^{0}\textrm{R}"/> is the global orientation of frame 3 which is computed given the value of `q1, q2, q3`, while <img src="https://latex.codecogs.com/gif.latex?\inline_{G}^{0}\textrm{R}"/> is the global orientation of gripper frame established by the Euler angle measured by ROS.

Comparing the symbolic and numerical value of <img src="https://latex.codecogs.com/gif.latex?\inline_{G}^{3}\textrm{R}"/> yields the equation which is sufficient for finding `q4, q5, q6`. Before  performing any further calculation, let's consider the simple case where `q5 = 0`. In this case, the orientation of the gripper frame is   

<img src="https://latex.codecogs.com/gif.latex?_{G}^{3}\textrm{R}&space;=&space;\begin{bmatrix}&space;\cos(q_4&plus;q_6)&space;&-\sin(q_4&plus;q_6)&space;&0&space;\\&space;0&space;&0&space;&1\\&space;-\sin(q_4&plus;q_6)&space;&-\cos(q_4&plus;q_6)&space;&0&space;\end{bmatrix}"/>     

This equation shows that the orientation of gripper frame does not explicitly depend on `q4` or `q6` individually, but depends on their sum. Therefore, as long as their sum satisfies the following equation, `q4` and `q6` can have any value.

<img src="https://latex.codecogs.com/gif.latex?q_4&plus;q_6&space;=&space;atan2(-_{G}^{3}\textrm{R}[0,1],&space;_{G}^{3}\textrm{R}[0,0])"/>

Turning to the generic case where `sin(q_5)` has nonzero value, the value of `q6` and `q4` is

<table><tr><td>
<img src="https://latex.codecogs.com/gif.latex?q_6&space;=&space;atan2(-_{G}^{3}\textrm{R}[1,1],&space;_{G}^{3}\textrm{R}[1,0])"/>
</td></tr></table>

<table><tr><td>
<img src="https://latex.codecogs.com/gif.latex?q_4&space;=&space;atan2(_{G}^{3}\textrm{R}[2,2],&space;-_{G}^{3}\textrm{R}[0,2])"/>
</td></tr></table>

The value of `q5` is defined based on the value of `q4`

<table><tr><td>
<img src="https://latex.codecogs.com/gif.latex?q_5&space;=&space;\left\{\begin{matrix}&space;atan2(-_{G}^{3}\textrm{R}[0,2]/\cos(q_4),\:&space;_{G}^{3}\textrm{R}[1,2]),&space;if\:&space;\sin(q_4)&space;=&space;0\\&space;atan2(_{G}^{3}\textrm{R}[2,2]/\sin(q_4),\:&space;_{G}^{3}\textrm{R}[0,2]),\:&space;otherwise&space;\end{matrix}\right."/>
</td></tr></table>

## 3. Project Implementation

The node which is in charge of performing Inverse Kinematics analysis is implemented in the file `IK_server.py`. This file begins with the definition of the `ik_helper_obj()` this a class that has all the gemetry definition of the robot as well as the methods supporting the IK analysis.

The attributes of the `ik_helper_obj` includes
* 4 list of symbolic variables denoting 4 type of DH parameters
* a dictionary storing the value of the already known DH parameters
* a list for symbolic variables representing the Euler angles of the gripper frame
Beside, this class has a method for constructing homogeneous transformation based 4 elements of a row of DH table (`homo_trans()`) and 3 methods respectively performing 3 element rotations (around x, y, and z axis). Finally, `ik_helper_obj` has two methods that support the angle calculation, namely `polarize_complex_num()` and `put_in_mp_pi()`. While the former transform a Cartesian complex number to polar form, the latter convert an arbitrari value of a angle to the range of [-pi, pi].

The process of deriving the position of 6 robot joints in the `handle_calculate_IK()` function is in the same flow as the Inverse Kinematic analysis section. Given the value of gripper frame position and its euler angles,
* 1st) calculate the gripper frame global orientation by multiplying the rotation matrix associated with each euler angle and perform the 2 body rotations to compensate for difference between URDF and DH convetion.
* 2nd) calculate the postion of the Wirst Center (WC) by substract the offset between the WC and the origin of the gripper frame.
* 3rd) find the position `ik_q1` of the first joint
* 4th) establish the complex form of the two equation of `q2` and `q3`, based on the `x` (or `y`) and `z` coordinate of the WC.
* 5th) solve these two equation for their values (`ik_q2` and `ik_q3`)
* 6th) calculate the orientation of the gripper frame relative to frame 3.
* 7th) derive the position of the last three joints through the orientation of the 6th step.

In the main function of `IK_server.py`, an instance of the `ik_helper_obj` is initialize (and named `ik_obj`). Then the symbollic form the homogeneous transformation from base frame to frame 3 (`symT_0_3`) and the global orientation of the gripper frame (`symR_0_G`) are derived to prepare the IK calculation in `handle_calculate_IK()` mentioned above. After this, the function `IK_server()` is invoked to put the node `IK_server` online.

Thanks to the good function of this node, robot has successfully performed the pick and place task. The footage of this task is shown in the figures below.

![alt text][image4]
*Fig.4 Robot reaches the target*

![alt text][image5]
*Fig.5 Robot retreive the target*

![alt text][image6]
*Fig.6 Robot reach the drop off position*

![alt text][image7]
*Fig.7 Robot release the target* 
