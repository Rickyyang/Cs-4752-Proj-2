#!/usr/bin/env python

import rospy
import argparse
import random
import baxter_external_devices
import numpy as np
from std_msgs.msg import String
import baxter_interface
import baxter_core_msgs
from baxter_interface import CHECK_VERSION
from baxter_interface import Limb
from baxter_pykdl import baxter_kinematics
import joint_velocity_wobbler as wb
import robot_rrt as rt
from geometry_msgs.msg import (
     PoseStamped,
     Pose,
     Point,
     Quaternion,
)

kin = None
wobbler = None
rate = None
spd = 0.03
hz = 3
omega = 5
W_1 = None
q_1 = None
left_arm = None


def create_sample_point():
	s0 = random.uniform(-141*np.pi/180 , 51.*np.pi/180)
	s1 = random.uniform(-123.*np.pi/180 , 60.*np.pi/180)
	e0 = random.uniform(-173.5*np.pi/180, 173.5*np.pi/180)
	e1 = random.uniform(-3.*np.pi/180 , 150.*np.pi/180)
	w0 = random.uniform(-175.25*np.pi/180, 175.25*np.pi/180)
	w1 = random.uniform(-90*np.pi/180 , 120*np.pi/180)
	w2 = random.uniform(-175.25*np.pi/180 , 175.25*np.pi/180)
	p = rt.MPoint(s0, s1, e0, e1, w0, w1, w2)
	return p

def draw_circle(n0, R01, theta, orientation="left", step=5*np.pi/180.,speed=0.03):
	global kin
	global wobbler
	global rate
	global omega
#	if orientation == "right":
#		step=-step
	t = theta/omega
	n_n = np.multiply(n0,speed)/np.sqrt(np.power(n0[0],2)+np.power(n0[1],2))
	n_norm = np.matrix([[n_n[0]],[n_n[1]]])
	for i in range(t-1):
		r_t = np.matrix([[np.cos(i*step), -np.sin(i*step)],[np.sin(i*step), np.cos(i*step)]])
		n = r_t * n_norm
		n1 = R01 * np.matrix([[n[0][0]],n[[1][0]],[0]])
		vx = n1[0]
		vy = n1[1]
		vz = n1[2]
		ksi = np.matrix([vx,vy,vz,[0],[0],[0]])
		pi = np.matrix(kin.jacobian_pseudo_inverse())
		b = second_object_manipul()								# compute second objective function
		j = np.matrix(kin.jacobian())
		v_joint = pi * ksi + (np.eye(7)-pi * j) * b 		# max mulnipulability
		#v_joint = pi * ksi 
		wobbler.wobble(v_joint)
		rate.sleep()

def draw_line(n0, R01, length):
	global kin
	global wobbler
	global rate
	global spd
	unit_length = 0.03
	t = int(length/unit_length)
	n_n = np.multiply(n0,spd)/np.sqrt(np.power(n0[0],2)+np.power(n0[1],2))
	n_norm = np.matrix([[n_n[0]],[n_n[1]]])
	for i in range(t-1):
		n1 = R01 * np.matrix([[n_norm[0][0]],n_norm[[1][0]],[0]])
		vx = n1[0]
		vy = n1[1]
		vz = n1[2]
		ksi = np.matrix([vx,vy,vz,[0],[0],[0]])
		pi = np.matrix(kin.jacobian_pseudo_inverse())
		b = second_object_manipul()								# compute second objective function
		j = np.matrix(kin.jacobian())
		v_joint = pi * ksi + (np.eye(7)-pi * j) * b 		# max mulnipulability
		#v_joint = pi * ksi
		wobbler.wobble(v_joint)
		rate.sleep()
			

def robot_interface():
	global kin
	global wobbler
	global rate
	global spd
	global hz
	global omega
	global left_arm

	# initial node
	cmd = raw_input ('Please input a command (p for proportional controller;\n l for going in line with a constant velocity;\n m for going in line with a constant velocity;\nr for doing rrt;\nc for doing current position;\n)\n')
	print "cmd=",cmd	
	rospy.init_node('robot_interface')
	time_start = rospy.Time.now()
	left_arm = Limb('left')
	kin = baxter_kinematics('left')
	kin.print_robot_description()
	kin.print_kdl_chain()
	# FK Position
	# FK Velocity
	# print '\n*** Baxter Velocity FK ***\n'
	# kin.forward_velocity_kinematics()
	# IK
	pos = [0.582583, -0.180819, 0.216003]
	rot = [0.03085, 0.9945, 0.0561, 0.0829]
	# Jacobian
	rate = rospy.Rate(10) # 10hz
	wobbler = wb.Wobbler() 
	start = wobbler._left_arm.endpoint_pose()
	start_pose = start['position']

	if cmd == "l":
		time_start = rospy.Time.now()
		end_pose = [0.75, 0, 0]
		vx = (end_pose[0] - start_pose[0])/10
		vy = (end_pose[1] - start_pose[1])/10
		vz = (end_pose[2] - start_pose[2])/10
		ksi = np.matrix([[vx],[vy],[vz],[0],[0],[0]])
		time = rospy.Time.now()-time_start
		while time.to_sec()<=5:
			time = rospy.Time.now()-time_start
			pi = np.matrix(kin.jacobian_pseudo_inverse())
			b = second_object_manipul()								# compute second objective function
			j = np.matrix(kin.jacobian())
			v_joint = pi * ksi + (np.eye(7)-pi * j) * b 		# max mulnipulability
			#v_joint = pi * ksi 
			wobbler.wobble(v_joint)
			p = wobbler._left_arm.endpoint_pose()
			pose = p['position']
			print pose
			rate.sleep()
	elif cmd == "p":
		time_start = rospy.Time.now()
		end_pose = [0.75, 0, 0]
		kp = 0.5
		time = rospy.Time.now()-time_start
		while time.to_sec()<=5:
			time = rospy.Time.now()-time_start
			p = wobbler._left_arm.endpoint_pose()
			pose = p['position']
			ex = end_pose[0] - pose[0]
			ey = end_pose[1] - pose[1]
			ez = end_pose[2] - pose[2]
			ksi = np.matrix([[kp*ex],[kp*ey],[kp*ez],[0],[0],[0]])
			pi = np.matrix(kin.jacobian_pseudo_inverse())
			b = second_object_manipul()								# compute second objective function
			j = np.matrix(kin.jacobian())
			v_joint = pi * ksi + (np.eye(7)-pi * j) * b 		# max mulnipulability
			#v_joint = pi * ksi 
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
		#build a frame on the new plane
		z_new = para.tolist()
		print "z_new: ", z_new
		z_new_n = z_new/np.sqrt(np.power(z_new[0][0],2)+np.power(z_new[1][0],2)+np.power(z_new[2][0],2))
		z_new_norm = [z_new_n[0][0], z_new_n[1][0], z_new_n[2][0]]
		x_new = [pose1[0]-pose2[0],pose1[1]-pose2[1],pose1[2]-pose2[2]]
		x_new_norm = x_new/np.sqrt(np.power(x_new[0],2)+np.power(x_new[1],2)+np.power(x_new[2],2))
		print x_new_norm, z_new_norm
		y_new_norm = np.cross(z_new_norm,x_new_norm)
		 # rotation matrix
		R_01 = np.matrix([[x_new_norm[0],y_new_norm[0],z_new_norm[0]],[x_new_norm[1],y_new_norm[1],z_new_norm[1]],[x_new_norm[2],y_new_norm[2],z_new_norm[2]]])
		#print "draw_cricle\n"
		#n0= [1.,0.]
		#draw_cricle(n0 , R_01, 360)
		#n0 = [1.,0.]
		# calculate the diameter of the circle
		d = spd*(1./hz)*(360./omega)/np.pi
		while True:
			cmd1 = raw_input('Please type in the character you want to writen\n')
			print "cmd1", cmd1
		
			if cmd1 == "A":
				# draw a A
				n0 = [1., 1.]
				draw_line(n0, R_01, 3*d)
				n0 = [1., -1.]
				draw_line(n0, R_01, 3*d)
				n0 = [-1.,1. ]
				draw_line(n0, R_01, 1*d)
				n0 = [-1., 0.]
				draw_line(n0, R_01, 2*d)
			elif cmd1 == "B":
				n0 = [1.,0.]
				# draw a B
				draw_circle(n0, R_01, 180)
				draw_circle(n0, R_01, 180)
				n0 = [0.,-1.]
				draw_line(n0, R_01, 2*d)
			elif cmd1 == "C":
				# draw a C
				n0 = [0.,1.]
				draw_circle(n0,R_01, 180)
				n0 = [0.,-1.]
				draw_line(n0, R_01, 1*d)
				n0 = [0.,-1.]
				draw_circle(n0, R_01, 180)
			elif cmd1 == "D":
				# Draw a D
				n0 = [1., 0.]
				draw_circle(n0, R_01, 180)
				n0 = [0., -1.]
				draw_line(n0, R_01, 1*d)

			elif cmd1 == "E":
				# Draw a E
				n0 = [-1., 0.]
				draw_line(n0, R_01, 0.5*d)
				n0 = [0., -1.]
				draw_line(n0, R_01, 0.5*d)
				n0 = [1., 0.]
				draw_line(n0, R_01, 0.5*d)
				n0 = [-1., 0.]
				draw_line(n0, R_01, 0.5*d)
				n0 = [0., -1.]
				draw_line(n0, R_01, 0.5*d)
				n0 = [1., 0.]
				draw_line(n0, R_01, 0.5*d)
			elif cmd1 == "O":
				# Draw a O
				n0 = [0., -1.]
				draw_circle(n0, R_01, 360)
			elif cmd1 == "X":
				# Draw a X
				n0 = [1., -1.]
				draw_line(n0, R_01, d)
				n0 = [-1., 1.]
				draw_line(n0, R_01, 0.5*d)
				n0 = [-1., -1.]
				draw_line(n0, R_01, d)
			elif cmd1 == "T":
				# Draw a T
				n0 = [1., 0.]
				draw_line(n0, R_01, d)
				n0 = [-1., 0.]
				draw_line(n0, R_01, 0.5*d)
				n0 = [0., -1.]
				draw_line(n0, R_01, 1.0*d)
			elif cmd1 == "R":
				# Draw a R
				n0 = [0., 1.]
				draw_line(n0, R_01, 0.5*d)
				n0 = [1., 0]
				draw_circle(n0, R_01, 180)
				n0 = [0., -1.]
				draw_line(n0, R_01, 0.5*d)
				n0 = [1., -1.]
				draw_line(n0, R_01, 0.5*1.41*d)
			elif cmd1 == "Baxter":
				# draw a B
				n0 = [1.,0.]
				draw_circle(n0, R_01, 180)
				draw_circle(n0, R_01, 180)
				n0 = [0.,-1.]
				draw_line(n0, R_01, 2*d)

				#Transition
				n0 = [1., 0.]
				draw_line(n0, R_01, 2*d)

				# draw an A
				n0 = [1., 1.]
				draw_line(n0, R_01, 2*1.41*d)
				n0 = [1., -1.]
				draw_line(n0, R_01, 2*1.41*d)
				n0 = [-1.,1.]
				draw_line(n0, R_01, 1.41/3*d)
				n0 = [-1., 0.]
				draw_line(n0, R_01, 2/3*d)

				#Transition

				n0 = [1., 0.]
				draw_line(n0, R_01, 2/3*d)
				n0 = [2., 1.]
				draw_line(n0, R_01, 2/3*d)

				# Draw a X
				n0 = [1., -1.]
				draw_line(n0, R_01, d)
				n0 = [-1., 1.]
				draw_line(n0, R_01, 0.5*d)
				n0 = [-1., -1.]
				draw_line(n0, R_01, d)

				#Transition

				n0 = [1., 0.]
				draw_line(n0, R_01, 0.5*d)

				# Draw a T
				n0 = [1., 0.]
				draw_line(n0, R_01, d)
				n0 = [-1., 0.]
				draw_line(n0, R_01, 0.5*d)
				n0 = [0., -1.]
				draw_line(n0, R_01, 1.0*d)

				#Transition

				n0 = [1., 0]
				draw_line(n0, R_01, 0.5*d)

				#Draw a E
				n0 = [-1., 0.]
				draw_line(n0, R_01, 0.5*d)
				n0 = [0., -1.]
				draw_line(n0, R_01, 0.5*d)
				n0 = [1., 0.]
				draw_line(n0, R_01, 0.5*d)
				n0 = [-1., 0.]
				draw_line(n0, R_01, 0.5*d)
				n0 = [0., -1.]
				draw_line(n0, R_01, 0.5*d)
				n0 = [1., 0.]
				draw_line(n0, R_01, 0.5*d)

				#Transition
				n0 = [1., 0]
				draw_line(n0, R_01, 0.5*d)

				#Draw a R
				n0 = [0., 1.]
				draw_line(n0, R_01, 0.5*d)
				n0 = [1., 0]
				draw_circle(n0, R_01, 180)
				n0 = [0., -1.]
				draw_line(n0, R_01, 0.5*d)
				n0 = [1., -1.]
				draw_line(n0, R_01, 0.5*1.41*d)
			elif cmd1 == "exit":
				break
			else:
				print "Wrong input"

	elif cmd == 'r':
		q_i = create_sample_point()
		q_g = create_sample_point()
		node_i = rt.Node(q_i, 0)
		node_i.add_node()
		node_g = rt.Node(q_g, 1)
		node_g.add_node()
		Path = rt.rrt(node_i, node_g)
		posi = rt.get_posi()
		rt.Draw_graph()
		print "Path: ", Path
		c = raw_input("Smoothing? y/n\n")
		if c == "y":
			Path = rt.smoothing()
			print "Path: ", Path
		print "posi: ", posi
		for i in range(len(Path)-1):
			print "the ", Path[i], "node's location is ", posi[Path[i]][0], posi[Path[i]][1], posi[Path[i]][2]
			x = posi[Path[i]][0] - posi[Path[i+1]][0]
			y = posi[Path[i]][1] - posi[Path[i+1]][1]
			z = posi[Path[i]][2] - posi[Path[i+1]][2]
			ksi = np.matrix([[3*x],[3*y],[3*z],[0],[0],[0]])
			pi = np.matrix(kin.jacobian_pseudo_inverse())
			v_joint = pi * ksi 
			wobbler.wobble(v_joint)
			rate.sleep()
		rt.Draw_graph()

	elif cmd == 't':
		xx = np.matrix(kin.jacobian()) * np.matrix(kin.jacobian_transpose())
		print "\n\n\n", xx
		
		

	elif cmd == "c":
		time = rospy.Time.now()-time_start
		while time.to_sec()<=20:
			time = rospy.Time.now()-time_start
			print "endpoint_pose: ", kin.forward_position_kinematics()

	else: 
		pass

def second_object_manipul():
	w =  np.sqrt(np.linalg.det(kin.jacobian() * kin.jacobian_transpose()))
	q = left_arm.joint_angles()

	ds0 = q
	ds0['left_s0'] = ds0['left_s0'] + 1
	w_ds0 = np.sqrt(np.linalg.det(kin.jacobian(ds0) * kin.jacobian_transpose(ds0)))
	dw_ds0 = w_ds0 - w

	ds1 = q
	ds1['left_s1'] = ds1['left_s1'] + 1
	w_ds1 = np.sqrt(np.linalg.det(kin.jacobian(ds1) * kin.jacobian_transpose(ds1)))
	dw_ds1 = w_ds1 - w

	de0 = q
	de0['left_e0'] = de0['left_e0'] + 1
	w_de0 = np.sqrt(np.linalg.det(kin.jacobian(de0) * kin.jacobian_transpose(de0)))
	dw_de0 = w_de0 - w

	de1 = q
	de1['left_e1'] = de1['left_e1'] + 1
	w_de1 = np.sqrt(np.linalg.det(kin.jacobian(de1) * kin.jacobian_transpose(de1)))
	dw_de1 = w_de1 - w

	dw0 = q
	dw0['left_s0'] = dw0['left_s0'] + 1
	w_dw0 = np.sqrt(np.linalg.det(kin.jacobian(dw0) * kin.jacobian_transpose(dw0)))
	dw_dw0 = w_dw0 - w

	dw1 = q
	dw1['left_s0'] = dw1['left_s0'] + 1
	w_dw1 = np.sqrt(np.linalg.det(kin.jacobian(dw1) * kin.jacobian_transpose(dw1)))
	dw_dw1 = w_dw1 - w

	dw2 = q
	dw2['left_s0'] = dw2['left_s0'] + 1
	w_dw2 = np.sqrt(np.linalg.det(kin.jacobian(dw2) * kin.jacobian_transpose(dw2)))
	dw_dw2 = w_dw2 - w	


	dw_dq = np.transpose(np.matrix([dw_ds0, dw_ds1, dw_de0, dw_de1, dw_dw0, dw_dw1, dw_dw2]))

	return dw_dq


# def second_objective():
# 	global kin
# 	global W_1
# 	global q_1
# 	if W_1 == None :
# 		W_1 = np.sqrt(np.linalg.dat( kin.Jacobian() , kin.Jacobian_transpose()))
# 		q_1 = wobbler._left_arm.joint_angles()               ### uncertain of this function
# 	q = wobbler._left_arm.joint_angles()					### uncertain of this function
# 	W = np.sqrt(np.linalg.dat( kin.Jacobian() , kin.Jacobian_transpose()))
# 	dW = np.subtract(W - W_1)
# 	dq = np.subtract(q - q_1)
# 	W_1 = W
# 	q_1 = q
# 	b = np.divide(dW,dq)
# 	b = np.transpose(b)
# 	return b

	
 
if __name__ == '__main__':
	try:
		rospy.loginfo('hello baxter!')
		robot_interface()
	except rospy.ROSInterruptException:
        	pass
