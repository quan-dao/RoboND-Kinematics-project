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


class ik_helper_obj():
    def __init__(self):
        # initilize symbolic variable representing DH parameters
        self.alpha_list = symbols('alpha0:7')  # links twist angle
        self.a_list = symbols('a0:7')  # links length
        self.d_list = symbols('d1:8')  # links offset
        self.q_list = symbols('q1:8')  # joints variable
        # define the DH table
        self.dh_table = {alpha0:    0,        a0:    0,         d1:    0.75,
                         alpha1:    -pi/2,    a1:    0.35,      d2:    0,         q2:    q2 - pi/2,
                         alpha2:    0,        a2:    1.25,      d3:    0,
                         alpha3:    -pi/2,    a3:    -0.054,    d4:    1.5,
                         alpha4:    pi/2,     a4:    0,         d5:    0,
                         alpha5:    -pi/2,    a5:    0,         d6:    0,
                         alpha6:    0,        a6:    0,         d7:    0.303,     q7:   0}
        self.euler_rpy = symbols('rpy0:3')  # gripper frame euler angles


    # Construct the homogeneous transformation based on 4 elements of a row of the DH table
    def homo_trans(self, alpha, a, d, theta):
        T_im1_i = Matrix([[cos(theta),                             -sin(theta),              0,                  a],
                          [sin(theta) * cos(alpha),    cos(theta) * cos(alpha),    -sin(alpha),    -sin(alpha) * d],
                          [sin(theta) * sin(alpha),    cos(theta) * sin(alpha),     cos(alpha),     cos(alpha) * d],
                          [0,                                                0,              0,                  1]])
        return T_im1_i.subs(self.dh_table)


    # Define elementary rotation
    def rot_x(self, roll):
        mat_rot_x = Matrix([[1,         0,          0],
                            [0, cos(roll), -sin(roll)],
                            [0, sin(roll), cos(roll)]])
        return mat_rot_x


    def rot_y(self, pitch):
        mat_rot_y = Matrix([[cos(pitch),     0,    sin(pitch)],
                            [0,              1,             0],
                            [-sin(pitch),    0,    cos(pitch)]])
        return mat_rot_y


    def rot_z(self, yaw):
        mat_rot_z = Matrix([[cos(yaw),    -sin(yaw),    0],
                            [sin(yaw),     cos(yaw),    0],
                            [0,                   0,    1]])
        return mat_rot_z


    # Angle calculating helper function
    def polarize_complex_num(self, re, img):
        moment = sqrt(re**2 + img**2)
        arg = atan2(img, re)
        return moment, arg


    def put_in_mp_pi(self, angle):
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


# The service handler function
def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
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
	        # Numerical value of global orientation of gripper frame
            ori_grip = symR_0_G.evalf(subs={ik_obj.euler_rpy[0]: roll, ik_obj.euler_rpy[1]: pitch, ik_obj.euler_rpy[2]: yaw})

            # Position of the Wrist Center
            wc_position = Matrix([[px], [py], [pz]]) - (0.303 + 0.1) * ori_grip[:, -1]


	        # IK decouple --> WC Position problem
            ik_q1 = atan2(wc_position[1], wc_position[0])  # angle of joint 1

            # the First equation of q2 and q3 is the Z coordinate of WC
            alpha = wc_position[2] - 0.75  # assign the RHS of the first equation of q2 & q3 to alpha
            # find the RHS of the Second equation of q2 and q3 from X or Y coordinate of WC and assign it to beta
            if sin(ik_q1) == 0:
                beta = wc_position[0] / cos(ik_q1) - 0.35
            else:
                beta = wc_position[1] / sin(ik_q1) - 0.35
	        # Complexize these 2 equation by  1st-eq + i * 2nd-eq
            r1, gamma1 = ik_obj.polarize_complex_num(alpha, beta)
            r, gamma = ik_obj.polarize_complex_num(-0.054, 1.5)  # coefficient of e^(i*(q2+q3))in the complexized equation

            # Geometrically solve for q2 & q3
            ik_q2 = gamma1 - acos((r1**2 + 1.25**2 - r**2) / (2 * r1 * 1.25))
            ik_q3 = gamma1 + asin(1.25 * sin(gamma1 - ik_q2) / r) - (ik_q2 + gamma)
            # Convert these angles to [-pi, pi]
            ik_q2 = ik_obj.put_in_mp_pi(ik_q2)
            ik_q3 = ik_obj.put_in_mp_pi(ik_q3)


            # IK decouple --> Orient the gripper
            # Calculate numerical value of pose of gripper
            T_0_3 = symT_0_3.evalf(subs={ik_obj.q_list[0]: ik_q1, ik_obj.q_list[1]: ik_q2, ik_obj.q_list[2]: ik_q3})
            # Orientation of gripper relative to frame{3}
            ori_3_G = (T_0_3[:-1, :-1]).T * ori_grip
            if abs(ori_3_G[1, 2]) == 1:  # i.e. sin(q5) == 0
                # orientation of gripper frame only depend on q4 + q6
                ik_q5 = 0
                sum46 = atan(-ori_3_G[0, 1], ori_3_G[0, 0])
                ik_q4 = 0.5 * sum46
                ik_q6 = 0.5 * sum46
            else:
                ik_q6 = atan2(-ori_3_G[1, 1], ori_3_G[1, 0])
                ik_q4 = atan2(ori_3_G[2, 2], -ori_3_G[0, 2])
                if sin(ik_q4) == 0:
                    ik_q5 = atan2(-ori_3_G[0, 2] / cos(ik_q4), ori_3_G[1, 2])
                else:
                    ik_q5 = atan2(ori_3_G[2, 2] / sin(ik_q4), ori_3_G[1, 2])
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
    # initialize IK helper object
    ik_obj = ik_helper_obj()
    # Symboliccaly calculate global pose of frame 3
    small_dh = zip(ik_obj.alpha_list, ik_obj.a_list, ik_obj.d_list, ik_obj.q_list, range(3))  # zip the first 3 rows of DH table
    symT_0_3 = eye(4)  # initialize symT_0_3
    for dh_row in small_dh:
        T_im1_i = ik_obj.homo_trans(dh_row[0], dh_row[1], dh_row[2], dh_row[3])
        symT_0_3 = symT_0_3 * T_im1_i
    # Symboliccaly calculate global pose of gripper frame
    symR_0_G = ik_obj.rot_z(ik_obj.euler_rpy[2]) * ik_obj.rot_y(ik_obj.euler_rpy[1]) * ik_obj.rot_x(ik_obj.euler_rpy[0]) * ik_obj.rot_y(pi/2) * ik_obj.rot_z(pi)
    IK_server()
