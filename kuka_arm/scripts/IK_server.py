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
T_0_G = Matrix([[1, 0, 0, 0],
                [0, 1, 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1]])  # initialize homogeneous transform representing pose of Gripper relative to base

T_0_3 = T_0_G  # pose of frame{3} relative to base link
T_0_4 = T_0_G  # pose of frame{4} relative to base link
i = 0  # index of rows of DH table
for dh_row in zip(alpha_list, a_list, d_list, q_list):
    T_im1_i = homo_trans(dh_row[0], dh_row[1], dh_row[2], dh_row[3])
    T_0_G = T_0_G * T_im1_i
    if i == 2:
        T_0_3 = T_0_G
    elif i == 3:
        T_0_4 = T_0_G
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
	    # Compensate for rotation discrepancy between DH parameters and Gazebo
	    #
	    #
	    # Calculate joint angles using Geometric IK method
	    #
	    #
            ###

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
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
