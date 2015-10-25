#!/usr/bin/env python

import rospy
import argparse
import baxter_external_devices
import numpy as np
from std_msgs.msg import String
import baxter_interface
import baxter_core_msgs
from baxter_interface import CHECK_VERSION
from baxter_pykdl import baxter_kinematics
import joint_velocity_wobbler as wb
from geometry_msgs.msg import (
     PoseStamped,
     Pose,
     Point,
     Quaternion,
)

def robot_interface():
    # initial node
        cmd = raw_input ('Please input a command (p for proportional controller;\n l for going in line with a constant velocity;\n; m for going in line with a constant velocity;\n)\n')
	print "cmd=",cmd	
	rospy.init_node('robot_interface')
	time_start = rospy.Time.now()
	print '*** Baxter PyKDL Kinematics ***\n'
	kin = baxter_kinematics('left')
	print '\n*** Baxter Description ***\n'
	kin.print_robot_description()
	print '\n*** Baxter KDL Chain ***\n'
	kin.print_kdl_chain()
	# FK Position
	print '\n*** Baxter Position FK ***\n'
	print kin.forward_position_kinematics()
	# FK Velocity
	# print '\n*** Baxter Velocity FK ***\n'
	# kin.forward_velocity_kinematics()
	# IK
	print '\n*** Baxter Position IK ***\n'
	pos = [0.582583, -0.180819, 0.216003]
	rot = [0.03085, 0.9945, 0.0561, 0.0829]
	print kin.inverse_kinematics(pos) # position, don't care orientation
	print '\n*** Baxter Pose IK ***\n'
	print kin.inverse_kinematics(pos, rot) # position & orientation
	# Jacobian
	print '\n*** Baxter Jacobian ***\n'
	print kin.jacobian()
	# Jacobian Transpose
	print '\n*** Baxter Jacobian Tranpose***\n'
	print kin.jacobian_transpose()
	# Jacobian Pseudo-Inverse (Moore-Penrose)
	print '\n*** Baxter Jacobian Pseudo-Inverse (Moore-Penrose)***\n'
	print kin.jacobian_pseudo_inverse()
	# Joint space mass matrix
	print '\n*** Baxter Joint Inertia ***\n'
	print kin.inertia()
	# Cartesian space mass matrix
	print '\n*** Baxter Cartesian Inertia ***\n'
	print kin.cart_inertia()
	rate = rospy.Rate(10) # 10hz
	wobbler = wb.Wobbler() 
	start = wobbler._left_arm.endpoint_pose()
	start_pose = start['position']
	if cmd == "l":
		end_pose = [0.75, 0, 0]
		vx = (end_pose[0] - start_pose[0])/10
		vy = (end_pose[1] - start_pose[1])/10
		vz = (end_pose[2] - start_pose[2])/10
		ksi = np.matrix([[vx],[vy],[vz],[0],[0],[0]])
		time = rospy.Time.now()-time_start
		while time.to_sec()<=10:
			time = rospy.Time.now()-time_start
			pi = np.matrix(kin.jacobian_pseudo_inverse())
			v_joint = pi * ksi 
			wobbler.wobble(v_joint)
			p = wobbler._left_arm.endpoint_pose()
			pose = p['position']
			print pose
			rate.sleep()
	elif cmd == "p":
		end_pose = [0.75, 0, 0]
		kp = 0.5
		time = rospy.Time.now()-time_start
		while time.to_sec()<=10:
			time = rospy.Time.now()-time_start
			p = wobbler._left_arm.endpoint_pose()
			pose = p['position']
			ex = end_pose[0] - pose[0]
			ey = end_pose[1] - pose[1]
			ez = end_pose[2] - pose[2]
			ksi = np.matrix([[kp*ex],[kp*ey],[kp*ez],[0],[0],[0]])
			pi = np.matrix(kin.jacobian_pseudo_inverse())
			v_joint = pi * ksi 
			wobbler.wobble(v_joint)
			print v_joint
			rate.sleep()
	elif cmd == "m":
		raw_input("Please move baxter to the first point...\n")
		p1 = wobbler._left_arm.endpoint_pose()
		pose1 = p1['position']
		raw_input("Please move baxter to the second point...\n")
		p2 = wobbler._left_arm.endpoint_pose()
		pose2 = p2['position']
		raw_input("Please move baxter to the third point...\n")
		p3 = wobbler._left_arm.endpoint_pose()
		pose3 = p3['position']
		print pose1,'\n', pose2,'\n', pose3,'\n'
		a = np.matrix([[pose1[0], pose1[1], pose1[2]],[pose2[0], pose2[1], pose2[2]],[pose3[0], pose3[1], pose3[2]]])
		d = np.matrix([[1], [1], [1]])
		para = np.linalg.solve(a, d)
		print para.tolist()
		x0 = (pose1[0]+pose2[0]+pose3[0])/3
		y0 = (pose1[1]+pose2[1]+pose3[1])/3
		z0 = (pose1[2]+pose2[2]+pose3[2])/3
		for i in range(10):
			y = y0 + 0.05*i
			zz = (1-para[0][0]*x0-para[1][0]*y)/para[2][0]
			z = zz[0][0]
			print i, y, z
			
		"""time = rospy.Time.now()-time_start
		while time.to_sec()<=10:
			time = rospy.Time.now()-time_start
			p = wobbler._left_arm.endpoint_pose()
			pose = p['position']
			ex = end_pose[0] - pose[0]
			ey = end_pose[1] - pose[1]
			ez = end_pose[2] - pose[2]
			ksi = np.matrix([[kp*ex],[kp*ey],[kp*ez],[0],[0],[0]])
			pi = np.matrix(kin.jacobian_pseudo_inverse())
			v_joint = pi * ksi 
			wobbler.wobble(v_joint)
			print v_joint
			rate.sleep()"""
	else:
		return 0
	
        # compute orintation of velocity 
        
	
'''	while not rospy.is_shutdown():
		# Jacobian Pseudo-Inverse (Moore-Penrose)
		#print '\n*** Baxter Jacobian Pseudo-Inverse (Moore-Penrose)***\n'
		#print kin.jacobian_pseudo_inverse()
		pi = np.matrix(kin.jacobian_pseudo_inverse())
		print pi * ksi 
		rate.sleep()'''
 
if __name__ == '__main__':
	try:
		rospy.loginfo('hello baxter!')
		robot_interface()
	except rospy.ROSInterruptException:
        	pass
