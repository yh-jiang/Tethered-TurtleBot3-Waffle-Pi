#!/usr/bin/env python3
from __future__ import print_function
import roslib; #roslib.load_manifest('teleop_twist_keyboard')
import rospy
#import Tkinter as tk
import time
import threading
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import sys, select, termios, tty

class trajectory_planning():
	def __init__(self):
		self.Traj    = np.loadtxt("traj_opt.txt", dtype='f', delimiter=',')
		self.x_old   = self.Traj[0,2] #self.Traj[0,1]
		self.y_old   = self.Traj[0,3] #self.Traj[0,2]
		self.rate	 = rospy.get_param('~rate', 45.0)
		self.step_num=0
		self.traj_pos=Twist()
		self.traj_vel=Twist()
		self.x=0.0
		self.y=0.0
		self.yaw=0.0
		self.pitch_dot=0.0
		self.last_update = rospy.get_time()
		self.traj_pub =rospy.Publisher('/traj_command_pos', Twist, queue_size = 1)
		self.start_signal_pub = rospy.Publisher('/start_signal', Float64, queue_size = 1)
		self.start_signal = Float64()
	def traj(self):	
		A=self.Traj[self.step_num,:]
		self.traj_pos.linear.x=A[2] # A[0]
		self.traj_pos.linear.y=A[3] # A[1]
		self.traj_pos.linear.z=0.0
		self.traj_pos.angular.x=(self.traj_pos.linear.x-self.x_old)*self.rate
		self.traj_pos.angular.y=(self.traj_pos.linear.y-self.y_old)*self.rate
		self.traj_pos.angular.z=0.0
		self.x_old = self.traj_pos.linear.x
		self.y_old = self.traj_pos.linear.y
	def step(self):
		if self.step_num < 2190:
			self.start_signal.data = 1.0
			self.traj()
			self.publish()
			self.step_num=self.step_num+1
		else:
			self.start_signal.data = 2.0
		self.start_signal_pub.publish(self.start_signal)
	def publish(self):
		#print(self.traj_pos)
		self.traj_pub.publish(self.traj_pos)
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
