#!/usr/bin/env python

# Copyright (C) 2017 Udacity Inc.
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


# Construct the homogeneous transformation based on 4 elements of a row of the DH table
def homo_trans(alpha, a, d, theta):
    T_im1_i = Matrix([[cos(theta),                             -sin(theta),              0,                  a],
                      [sin(theta) * cos(alpha),    cos(theta) * cos(alpha),    -sin(alpha),    -sin(alpha) * d],
                      [sin(theta) * sin(alpha),    cos(theta) * sin(alpha),     cos(alpha),     cos(alpha) * d],
                      [0,                                                0,              0,                  1]])
    return T_im1_i.subs(s)


# Define elementary rotation
def rot_x(roll):
    mat_rot_x = Matrix([[1,         0,          0],
                        [0, cos(roll), -sin(roll)],
                        [0, sin(roll), cos(roll)]])
    return mat_rot_x.evalf()


def rot_y(pitch):
    mat_rot_y = Matrix([[cos(pitch),     0,    sin(pitch)],
                        [0,              1,             0],
                        [-sin(pitch),    0,    cos(pitch)]])
    return mat_rot_y.evalf()


def rot_z(yaw):
    mat_rot_z = Matrix([[cos(yaw),    -sin(yaw),    0],
                        [sin(yaw),     cos(yaw),    0],
                        [0,                   0,    1]])
    return mat_rot_z.evalf()


# Angle calculating helper function
def polarize_complex_num(re, img):
    moment = sqrt(re**2 + img**2)
    arg = atan2(img, re)
    return moment, arg


def put_in_mp_pi(angle):
    # put the angle in the range [-pi, pi]
    while abs(angle) >= (2 * pi):
        if angle > 0:
            angle -= 2 * pi
        else:
            angle += 2 * pi
    if angle > pi:
        angle -= 2 * pi
    elif angle < -pi:
        angle += 2 *  pi
    return angle.evalf()

# Create symbols
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')  # joint twist angle
a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')  # links length
d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')  # links offset
q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')  # joints variable

# Define dictianary storing the DH table
s = {alpha0:    0,        a0:    0,         d1:    0.75,
     alpha1:    -pi/2,    a1:    0.35,      d2:    0,         q2:    q2 - pi/2,
     alpha2:    0,        a2:    1.25,      d3:    0,
     alpha3:    -pi/2,    a3:    -0.054,    d4:    1.5,
     alpha4:    pi/2,     a4:    0,         d5:    0,
     alpha5:    -pi/2,    a5:    0,         d6:    0,
     alpha6:    0,        a6:    0,         d7:    0.303,     q7:   0}

# Symbolically calculate homogeneous transformation matrix
symT_0_3 = Matrix([[1, 0, 0, 0],
                [0, 1, 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1]])  # initialize homogeneous transform representing pose of frame{3} relative to base

i = 0  # index of rows of DH table
for dh_row in zip(alpha_list, a_list, d_list, q_list):
    sym_T_im1_i = homo_trans(dh_row[0], dh_row[1], dh_row[2], dh_row[3])  # a symbolic matrix because of q_list
    symT_0_3 = symT_0_3 * symT_im1_i
    if i == 2:
        break;
    i += 1


# The service handler function
def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

        ### Your FK code here
        # Create symbols
	#
	#
	# Create Modified DH parameters
	#
	#
	# Define Modified DH Transformation matrix
	#
	#
	# Create individual transformation matrices
	#
	#
	# Extract rotation matrices from the transformation matrices
	#
	#
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

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])

            ### Your IK code here
	    # Orientation of Gripper frame relative to base frame
            ori_grip = rot_z(yaw) * rot_y(pitch) * rot_x(roll)  # orientation of Gripper by Euler angle
            ori_grip = ori_grip * rot_y(pi/2) * rot_z(pi)  # make Gripper frame coincident with DH convention

            # Position of the Wrist Center
            wc_position = Matrix([[px], [py], [pz]]) - (0.303 + 0.0375) * ori_grip[:, -1]


	    # IK decouple --> WC Position problem
            ik_q1 = atan2(wc_position[1], wc_position[0])  # angle of joint 1

            # the First equation of q2 and q3 is the Z coordinate of WC
            alpha = wc_position[2] - 0.75  # assign the RHS of this equation to alpha
            # find the RHS of the Second equation of q2 and q3 from X or Y coordinate of WC and assign it to beta
            if sin(ik_q1) == 0:
                beta = wc_position[0] / cos(ik_q1) - 0.35
            else:
                beta = wc_position[1] / sin(ik_q1) - 0.35
	    # Complexize these 2 equation by  1st-eq + i * 2nd-eq
            r1, gamma1 = polarize_complex_num(alpha, beta)
            r, gamma = polarize_complex_num(-0.054, 1.5)  # coefficient of e^(i*(q2+q3))in the complexized equation

            # Geometrically solve for q2 & q3
            ik_q2 = gamma1 - acos((r1**2 + 1.25**2 - r**2) / (2 * r1 * 1.25))
            ik_q3 = gamma1 + asin(1.25 * sin(gamma1 - ik_q2) / r) - (ik_q2 + gamma)
            # Convert these angles to [-pi, pi]
            ik_q2 = put_in_mp_pi(ik_q2)
            ik_q3 = put_in_mp_pi(ik_q3)


            # IK decouple --> Orient the gripper
            T_0_3 = symT_0_3.evalf(subs={q1: ik_q1, q2: ik_q2, q3: ik_q3})  # numerical value of pose of gripper
            # relative to base frame give value of the first 3 joints
            ori_3_G = (T_0_3[:-1, :-1]).T * ori_grip   # orientation of gripper relative to frame{3}
            if abs(T_3_G[1, 2]) == 1:  # i.e. sin(q5) == 0
                # orientation of gripper frame only depend on q4 + q6
                ik_q5 = 0
                sum46 = atan(-T_3_G[0, 1], T_3_G[0, 0])
                ik_q4 = 0.5 * sum46
                ik_q6 = 0.5 * sum46
            else:
                ik_q6 = atan2(-T_3_G[1, 1], T_3_G[1, 0])
                ik_q4 = atan2(T_3_G[2, 2], -T_3_G[0, 2])
                if sin(ik_q4) == 0:
                    ik_q5 = atan2(-T_3_G[0, 2] / cos(ik_q4), T_3_G[1, 2])
                else:
                    ik_q5 = atan2(T_3_G[2, 2] / sin(ik_q4), T_3_G[1, 2])
            ###

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
	    joint_trajectory_point.positions = [ik_q1, ik_q2, ik_q3, ik_q4, ik_q5, ik_q6]
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
