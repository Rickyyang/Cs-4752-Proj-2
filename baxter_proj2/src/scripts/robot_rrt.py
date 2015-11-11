#!/usr/bin/env python

import random
import rospy
import struct
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import robot_interface as robot
from collision_checker.srv import CheckCollision
import baxter_interface
from std_msgs.msg import String
import copy


location = []
e = 1.5
q_new = None 
node_ID = 0
connect_ID = None
Con_node = []
final_ID = None
kin =None
init_limb_pos = None
posi = []
Path = []

class MPoint:
	def __init__(self, s0, s1, e0, e1, w0, w1, w2):
		self.s0 = s0
		self.s1 = s1
		self.e0 = e0
		self.e1 = e1
		self.w0 = w0
		self.w1 = w1
		self.w2 = w2
	
class Node:
	global location
	def __init__ (self, p, t_ID=None):
		global node_ID
		self.ID = node_ID
		node_ID = node_ID + 1
		self.s0 = p.s0
		self.s1 = p.s1
		self.e0 = p.e0
		self.e1 = p.e1
		self.w0 = p.w0
		self.w1 = p.w1
		self.w2 = p.w2
		self.t_ID = t_ID

	def add_node(self):
		loc = [self.ID, self.s0, 
		       self.s1, self.e0, 
		       self.e1, self.w0, 
		       self.w1, self.w2, 
		       self.t_ID]
		location.append(loc)		

	def add_connect(self, ID): 
		global Con_node
		Con_node.append(ID)

def collision_check(q):
	#print "collision_check!!"
	#print "q: ", q.s0, q.s1, q.e0, q.e1, q.w0, q.w1, q.w2
	q_list = [q.s0, q.s1, q.e0, q.e1, q.w0, q.w1, q.w2]
	service = rospy.ServiceProxy('/check_collision', CheckCollision)
	rospy.wait_for_service('/check_collision')
	res = service(String('left'), q_list)
	print "collision result: ", res.collision
	return res.collision

def collision_check_line(q1, q2):
	q_1 = np.array([q1.s0,q1.s1,q1.e0,q1.e1,q1.w0,q1.w1,q1.w2])
	q_2 = np.array([q2.s0,q2.s1,q2.e0,q2.e1,q2.w0,q2.w1,q2.w2])
	dis = np.subtract(q_1, q_2)
	n_1 = np.linalg.norm(dis)
	norm = dis/n_1
	n = int(n_1)
	print "collision_check_line!!"
	for i in range(n):
		q = np.add(q_2, i*norm)
		qq = q.tolist()
		p = MPoint(qq[0],qq[1],qq[2],qq[3],qq[4],qq[5],qq[6])
		if collision_check(p):
			return True
	return False


def distance (q, pos):
	return np.sqrt((q.s0-pos[1])*(q.s0-pos[1])+
		           (q.s1-pos[2])*(q.s1-pos[2])+
		           (q.e0-pos[3])*(q.e0-pos[3])+
		           (q.e1-pos[4])*(q.e1-pos[4])+
		           (q.w0-pos[5])*(q.w0-pos[5])+
		           (q.w1-pos[6])*(q.w1-pos[6])+
		           (q.w2-pos[7])*(q.w2-pos[7]))

def nearest_node(t_ID, q, q_ID=None):
	min = 10000
	for i in range(len(location)):
		if t_ID == 0 and location[i][8] == 1: 
			continue
		if t_ID == 1 and location[i][8] == 0: 
			continue
		if i == q_ID:
			continue
		d = distance(q, location[i])
		if i == 0 or min > d:
			min = d
			min_ID = i
	return min_ID

def add_point(q, q_near):
	global e
	global q_new
	l = np.sqrt((q.s0-q_near.s0)*(q.s0-q_near.s0)+
				(q.s1-q_near.s1)*(q.s1-q_near.s1)+
				(q.e0-q_near.e0)*(q.e0-q_near.e0)+
				(q.e1-q_near.e1)*(q.e1-q_near.e1)+
				(q.w0-q_near.w0)*(q.w0-q_near.w0)+
				(q.w1-q_near.w1)*(q.w1-q_near.w1)+
				(q.w2-q_near.w2)*(q.w2-q_near.w2))
	if l == 0:
		l = 1
	if l <= e:
		q_new = q
		return q_new
	rate = e / l
	q_new = MPoint(rate*(q.s0-q_near.s0)+q_near.s0,
				  rate*(q.s1-q_near.s1)+q_near.s1,
				  rate*(q.e0-q_near.e0)+q_near.e0,
				  rate*(q.e1-q_near.e1)+q_near.e1,
				  rate*(q.w0-q_near.w0)+q_near.w0,
				  rate*(q.w1-q_near.w1)+q_near.w1,
				  rate*(q.w2-q_near.w2)+q_near.w2)
	return q_new

def node_extend(t, q, con_ID=None):
	global location
	global q_new
	global connect_ID
	global final_ID
	min_ID = nearest_node(t.ID, q, con_ID)
	#print "node_extend t_ID: ", t.ID
	q_near = MPoint(location[min_ID][1], 
		           location[min_ID][2], 
		           location[min_ID][3], 
		           location[min_ID][4], 
		           location[min_ID][5], 
		           location[min_ID][6], 
		           location[min_ID][7])
	q_new = add_point(q, q_near)
	if not collision_check_line(q_near, q_new):
		node = Node(q_new, t.ID)
		node.add_node()
		node.add_connect(min_ID)
		connect_ID = node.ID
		if q_new == q:
			final_ID = con_ID
			#print "con_ID: ", con_ID
			return "Reached"
		else:
			return "Advanced"
	return "Trapped"


def node_connect(t, q, con_ID):
	s = "Advanced"
	while s == "Advanced": 
		s = node_extend(t, q, con_ID)
	return s

def create_sample_point():
	e0 = random.uniform(-141.*np.pi/180 , 51.*np.pi/180)
	s1 = random.uniform(-123.*np.pi/180 , 60.*np.pi/180)
	s0 = random.uniform(-173.5*np.pi/180, 173.5*np.pi/180)
	e1 = random.uniform(-3.*np.pi/180 , 150.*np.pi/180)
	w0 = random.uniform(-175.25*np.pi/180, 175.25*np.pi/180)
	w1 = random.uniform(-90*np.pi/180 , 120*np.pi/180)
	w2 = random.uniform(-175.25*np.pi/180 , 175.25*np.pi/180)
	p = MPoint(s0, s1, e0, e1, w0, w1, w2)
	return p
	#return {'left_s0': s0, 'left_s1':s1, 'left_e0' : e0,'left_e1': e1, 'left_w0' : w0, 'left_w1' : w1, 'left_w2' : w2}

def rrt(node_i, node_g):
	global location
	global q_new
	global fig
	global connect_ID
	global ax
	global Con_node
	global final_ID
	global kin
	global init_limb_pos	
	global Path
	kin = robot.baxter_kinematics('left')
	k = 2000
	# sampling!
	for i in range(k):
		# get a sample point
		joint_rand = create_sample_point()
		#print "joint_rand: ", joint_rand.s0, joint_rand.s1, joint_rand.e0, joint_rand.e1, joint_rand.w0, joint_rand.w1, joint_rand.w2
		if collision_check(joint_rand):
			continue
		if not node_extend(node_i, joint_rand) == "Trapped":
			if node_connect(node_g, q_new, connect_ID) == "Reached":
				print "Finish!"
				break
	print "random point number: ", i
	print "node number: ", len(location)
	Path0 = []
	Path1 = []
	next = final_ID
	#print "final_ID: ", final_ID
	Path0.append(next)
	while not next == 0:
		#print "next: ", next
		next = Con_node[next-2]
		Path0.append(next)
	next = len(location)-1
	Path1.append(next)
	while not next == 1:
		next = Con_node[next-2]
		Path1.append(next)
	Path0.reverse()
	Path = Path0 + Path1
	return Path

def get_posi():
	global posi
	global location
	for i in range(len(location)):
		joints = {'left_s0': location[i][1], 'left_s1':location[i][2], 'left_e0' : location[i][3],'left_e1': location[i][4], 'left_w0' : location[i][5], 'left_w1' : location[i][6], 'left_w2' : location[i][7]}
		#print "joints: ", joints
		posi.append(kin.forward_position_kinematics(joints))	
	return posi

def Draw_graph():
	global location
	global kin
	global posi
	global Path
	x_p = []
	y_p = []
	z_p = []
	fig = plt.figure()
	ax = fig.add_subplot(111, projection='3d')
	for i in range(len(Path)):
		x_p.append(posi[Path[i]][0])
		y_p.append(posi[Path[i]][1])
		z_p.append(posi[Path[i]][2])
		ax.scatter(x_p, y_p, z_p)
	for i in range(len(Path)-1):
		x1 = posi[Path[i]][0]
		x2 = posi[Path[i+1]][0]
		y1 = posi[Path[i]][1]
		y2 = posi[Path[i+1]][1]
		z1 = posi[Path[i]][2]
		z2 = posi[Path[i+1]][2]
		ax.plot([x1, x2], [y1, y2], [z1, z2])
	plt.show()

def smoothing():
	global location
	global final_ID
	global Path
	global Con_node
	i = 0
	#out = copy.copy(Path)
	while location[Path[i]][8] != 1:
		i = i + 1
	start = i - 1
	for i in range(start, -1, -1):
		if i+2 > len(Path):
			break
		for j in range(i+2, len(Path), 1):
			print "i: ", i, "j: ", j, "Path: ", len(Path)
			q1 = MPoint(location[Path[i]][1], 
			           location[Path[i]][2], 
			           location[Path[i]][3], 
			           location[Path[i]][4], 
			           location[Path[i]][5], 
			           location[Path[i]][6], 
			           location[Path[i]][7])
			q2 = MPoint(location[Path[j]][1], 
			           location[Path[j]][2], 
			           location[Path[j]][3], 
			           location[Path[j]][4], 
			           location[Path[j]][5], 
			           location[Path[j]][6], 
			           location[Path[j]][7])
			if not collision_check_line(q1, q2):
				Path.pop(j-1)
			else: 
				break
			if j >= len(Path)-1:
				break
	return Path



def main():
	global location
	global Con_node
	# initial a start point and end point 
	
	q_i = MPoint(0., 0., 0.)
	q_g = MPoint(1., 1., 1.)
	node_i = Node(q_i, 0)
	node_i.add_node()
	node_g = Node(q_g, 1)
	node_g.add_node()
	rrt(node_i, node_g)
	#print "location: ", location


# if python says run, then we should run
if __name__ == '__main__':
	main()


