#!/usr/bin/env python3
from __future__ import print_function
import roslib; #roslib.load_manifest('teleop_twist_keyboard')
import rospy
#import Tkinter as tk
import time
import threading
import numpy as np
from geometry_msgs.msg import Twist
import sys, select, termios, tty

class trajectory_planning():
	def __init__(self):
		self.Traj    = np.loadtxt("traj.txt", dtype='f', delimiter=',')
		self.x_old_1   = self.Traj[0,0]
		self.y_old_1   = self.Traj[0,1]
		self.x_old_2   = self.Traj[0,2]
		self.y_old_2   = self.Traj[0,3]
		self.rate	 = rospy.get_param('~rate', 100.0)
		self.step_num=0
		self.traj_pos_1=Twist()
		self.traj_vel_1=Twist()
		self.traj_pos_2=Twist()
		self.traj_vel_2=Twist()
		# self.x=0.0
		# self.y=0.0
		self.yaw=0.0
		self.pitch_dot=0.0
		self.last_update = rospy.get_time()
		self.traj_pub_1 =rospy.Publisher('/tethered_turtle_sim/turtle1/traj_command_pos',Twist,queue_size=1)
		self.traj_pub_2 =rospy.Publisher('/tethered_turtle_sim/turtle2/traj_command_pos',Twist,queue_size=1)
	def traj(self):	
		A=self.Traj[self.step_num,:]
		self.traj_pos_1.linear.x=A[0] #x
		self.traj_pos_1.linear.y=A[1]   #y
		self.traj_pos_1.linear.z=0.0
		self.traj_pos_1.angular.x=(self.traj_pos_1.linear.x-self.x_old_1)*self.rate
		self.traj_pos_1.angular.y=(self.traj_pos_1.linear.y-self.y_old_1)*self.rate
		self.traj_pos_1.angular.z=0.0
		self.x_old_1 = self.traj_pos_1.linear.x
		self.y_old_1 = self.traj_pos_1.linear.y

		self.traj_pos_2.linear.x=A[2] #x
		self.traj_pos_2.linear.y=A[3]   #y
		self.traj_pos_2.linear.z=0.0
		self.traj_pos_2.angular.x=(self.traj_pos_2.linear.x-self.x_old_2)*self.rate
		self.traj_pos_2.angular.y=(self.traj_pos_2.linear.y-self.y_old_2)*self.rate
		self.traj_pos_2.angular.z=0.0
		self.x_old_2 = self.traj_pos_2.linear.x
		self.y_old_2 = self.traj_pos_2.linear.y

	def step(self):
		self.traj()
		self.publish()
		self.step_num=self.step_num+1
	def publish(self):
		#print(self.traj_pos)
		self.traj_pub_1.publish(self.traj_pos_1)
		self.traj_pub_2.publish(self.traj_pos_2)
	def run(self):
		rate =rospy.Rate(self.rate)
		while not rospy.is_shutdown():
			self.step()	
			rate.sleep()

if __name__ == '__main__':
	rospy.init_node('trajectory_planning')
	try:
		obj = trajectory_planning().run()
	except:
		pass
