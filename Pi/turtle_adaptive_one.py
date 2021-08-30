#!/usr/bin/env python3
import numpy as np 
import time
import threading
import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from std_msgs.msg import Float64
# from gazebo_msgs.msg import LinkStates
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_matrix
from tf.transformations import quaternion_from_matrix
from tf.transformations import quaternion_multiply
from tf.transformations import quaternion_about_axis
from tf.transformations import euler_matrix
rad2degrees = 180.0/math.pi
degrees2rad = math.pi / 180.0


class rotor_combined_controller():
	"""docstring for rotor_combined_controller"""
	def __init__(self):
		self.rate		= rospy.get_param('~rate',45.0)
		self.m	 		= 1.5
		self.J 			= 0.146
		self.r			= 0.033
		self.R			= 0.144
		self.l 		  	= 0.02
		self.d 			= 0.15
		self.J			= self.J+self.m*self.l**2

		# Start Signal
		self.start_signal = 0.0

		# Control Inputs
		self.quaternion		= np.array([0.0,0.0,0.0,1.0])
		self.pos_ref 		= np.matrix([0.0,0.0]).transpose()
		self.vel_ref 		= np.matrix([0.0,0.0]).transpose()


		self.pos_mea    	= np.matrix([0.0,0.0,0.0]).transpose()
		self.vel_mea 		= np.matrix([0.0,0.0,0.0]).transpose()
		self.v_d			= np.matrix([0.0, 0.0]).transpose()
		self.v_d_old	    = np.matrix([0.0, 0.0]).transpose()
		self.v_d_dot 		= np.matrix([0.0, 0.0]).transpose()

		# Controller_outputs
		self.dt 			= 0.01
		self.torque_1		=Float64()
		self.torque_2		=Float64()
		self.torque_3		=Float64()
		self.torque_4		=Float64()
		self.step_num	    =0
		#pubs
		self.joint_pb1	=rospy.Publisher('/cmd_torque_left',Float64,queue_size=1)
		self.joint_pb2	=rospy.Publisher('/cmd_torque_right',Float64,queue_size=1)
		#self.joint_pb3	=rospy.Publisher('/crazyjack/tilt_joint_3/command',Float64,queue_size=1)
		#self.joint_pb4	=rospy.Publisher('/crazyjack/tilt_joint_4/command',Float64,queue_size=1)
		#subs
	    #self.sub1	=rospy.Subscriber('/crazyjack/ground_truth/odometry',Odometry,self.odometry_process,queue_size=1)
		self.linkstate  =rospy.Subscriber('/odom',Odometry,self.data_process_link,queue_size=1)
		self.traj_sub   =rospy.Subscriber('/traj_command_pos',Twist,self.update_setpoints_pos,queue_size=1)
		self.start_signal_sub = rospy.Subscriber('/start_signal',Float64,self.update_start_signal, queue_size = 1)
		#self.traj_sub2  =rospy.Subscriber('/turtlebot3_waffle_pi_sim/traj_command_vel',Twist,self.update_setpoints_vel,queue_size=1)
		
		#self.testdata_1         = Float64()
		#self.testdata_2         = Float64()
		#self.testdata_pub_1     = rospy.Publisher('/test_data_pub_1', Float64, queue_size = 1)
		#self.testdata_pub_2     = rospy.Publisher('/test_data_pub_2', Float64, queue_size = 1)

		self.torque_1_last = Float64()
		self.torque_2_last = Float64()

		self.f = open('real_traj.txt', 'w')
	def data_process_link(self,Msg):
		self.pos_mea[0,0]  = -Msg.pose.pose.position.y + 1
		self.pos_mea[1,0]  = Msg.pose.pose.position.x + 0.5
		self.f.write(str(self.pos_mea[0,0]) + ',' + str(self.pos_mea[1,0]))
		self.f.write('\n')
		self.quaternion[0] = Msg.pose.pose.orientation.x
		self.quaternion[1] = Msg.pose.pose.orientation.y
		self.quaternion[2] = Msg.pose.pose.orientation.z
		self.quaternion[3] = Msg.pose.pose.orientation.w
		#self.testdata_1.data = self.pos_mea[0,0]
		rpy=euler_from_quaternion(self.quaternion,axes='sxyz')
		self.pos_mea[2,0] =rpy[2] + 1.57#np.arctan2(2 * (self.quaternion[0] * self.quaternion[3] + self.quaternion[1] * self.quaternion[2]),
		#(1 - 2 * (self.quaternion[2] * self.quaternion[2] + self.quaternion[3] * self.quaternion[3])))
		#print(self.quaternion_1)
		#self.testdata_2.data  = self.pos_mea[2,0]
		self.vel_mea[0,0]	  = -Msg.twist.twist.linear.y
		self.vel_mea[1,0]	  = Msg.twist.twist.linear.x
		self.vel_mea[2,0]	  =	Msg.twist.twist.angular.z
		#print(self.vel_mea)
	def update_control(self):
		t=self.pos_mea[2,0]
		t_dot=self.vel_mea[2,0]
		S=np.mat([[np.cos(t), -self.l*np.sin(t)],[np.sin(t), self.l*np.cos(t)],[0,1]])
		S_dot=np.mat([[-np.sin(t)*t_dot, -self.l*np.cos(t)*t_dot],[np.cos(t)*t_dot, -self.l*np.sin(t)*t_dot],[0.0,0.0]])
		self.vr=np.linalg.pinv(S)*self.vel_mea

		M=np.mat([[self.m, 0.0, self.m*self.l*np.sin(t)],[0, self.m, -self.m*self.l*np.cos(t)],[self.m*self.l*np.sin(t), -self.m*self.l*np.cos(t),self.J]])
		Vm=np.mat([[0.0,0.0,self.m*self.l*t_dot*np.cos(t)],[0.0,0.0,self.m*self.l*t_dot*np.sin(t)],[0.0,0.0,0.0]])
		E=1/self.r*np.mat([[np.cos(t),np.cos(t)],[np.sin(t),np.sin(t)],[self.R,-self.R]])
		M_bar=S.transpose()*M*S
		V_bar=S.transpose()*M*S_dot+S.transpose()*Vm*S
		E_bar=S.transpose()*E
		self.tau=np.linalg.inv(E_bar)*(M_bar*self.v_d_dot+V_bar*self.v_d)
		K = np.matrix([[3.5, 0.0],[0.0,1.0]])
		#e=self.v_d-self.vr
		uk=K*self.v_d-K*self.vr
		u=M_bar*(uk+self.v_d_dot)+V_bar*self.vr
		self.tau=np.linalg.inv(E_bar)*u
		self.tau=np.clip(self.tau,-8,8)
		print(self.v_d,self.vr,self.tau)
	def update_setpoints_pos(self,trajMsg):
		self.pos_ref = np.matrix([trajMsg.linear.x, trajMsg.linear.y]).transpose()     #np.asarray(pos_setpoint)
		self.vel_ref = np.matrix([trajMsg.angular.x,trajMsg.angular.y]).transpose()
	#def update_setpoints_vel(self,trajMsg):

	def update_start_signal(self, startMsg):
		self.start_signal = startMsg.data

	def pos_controller_step(self):
		#self.pos_ref = np.matrix([0.1, 0.2]).transpose()
		#self.vel_ref = np.matrix([0.2, 0.3]).transpose()
		err_x = self.pos_ref[0,0] - self.pos_mea[0,0]
		err_y = self.pos_ref[1,0] - self.pos_mea[1,0]
		phi   = self.pos_mea[2,0]
		lx=20.0
		ly=20.0
		kx=5.0
		ky=5.0
		xx	=self.vel_ref[0,0] +lx*np.tanh(kx/lx*err_x)
		yy	=self.vel_ref[1,0] +ly*np.tanh(ky/ly*err_y)
		K	=np.matrix([[np.cos(phi),np.sin(phi)],[-1/self.d*np.sin(phi), 1/self.d*np.cos(phi)]])

		S = np.mat([[np.cos(phi), -self.d * np.sin(phi)], [np.sin(phi), self.d * np.cos(phi)]])
		K2=np.linalg.pinv(S)
		#print(K,K2)
		self.v_d = K*np.mat([xx,yy]).transpose()
		if (self.step_num<1):
			self.v_d_dot=np.mat([0,0]).transpose()
			self.v_d_old=self.v_d
		else:
			self.v_d_dot = (self.v_d - self.v_d_old) / self.dt
		self.v_d_old=self.v_d
		self.step_num=self.step_num+1
		self.v_d_dot[0,0] = np.clip(self.v_d_dot[0,0], -8, 8)
		self.v_d_dot[1,0] = np.clip(self.v_d_dot[1,0], -20, 20)
		print(1,self.pos_ref,self.pos_mea,self.vel_ref,self.v_d_dot)


	def step(self):
		#if self.start_signal == 1.0:

		self.dt=1/self.rate
		self.pos_controller_step()
		self.update_control()
		#print(self.thrust)
		#print(self.beta)
		#time.sleep(0.02)		
		self.publish_command()
		
	def publish_command(self):
		self.torque_1.data=self.tau[1,0]
		self.torque_2.data=self.tau[0,0]
		#self.thrust_3.data=-1*self.thrust[2]
		#self.thrust_4.data=-1*self.thrust[3]
		self.joint_pb1.publish(self.torque_1)
		self.joint_pb2.publish(self.torque_2)
		#self.thrust_pb3.publish(self.thrust_3)
		#self.thrust_pb4.publish(self.thrust_4)
		# 
		#self.testdata_pub_1.publish(self.testdata_1)
		#self.testdata_pub_2.publish(self.testdata_2)
		self.torque_1_last = self.torque_1
		self.torque_2_last = self.torque_2

	def run(self):
		rate =rospy.Rate(self.rate)
		#while(self.start_signal != 1.0):
		#	self.torque_1.data=0.0
		#	self.torque_2.data=0.0
		#	self.joint_pb1.publish(self.torque_1)
		#	self.joint_pb2.publish(self.torque_2)
		#while not rospy.is_shutdown():
		#	if(self.start_signal == 1.0):
		#		self.step()	
		#		rate.sleep()
		#	else:
		#		self.torque_1.data=0.0
		#		self.torque_2.data=0.0
		#		self.joint_pb1.publish(self.torque_1)
		#		self.joint_pb2.publish(self.torque_2)

		while not rospy.is_shutdown():
			if (self.start_signal == 0.0):
				self.torque_1.data=0.0
				self.torque_2.data=0.0
				self.joint_pb1.publish(self.torque_1)
				self.joint_pb2.publish(self.torque_2)
			elif(self.start_signal == 1.0):
				self.step()	
				rate.sleep()
			else:
				self.joint_pb1.publish(self.torque_1_last)
				self.joint_pb2.publish(self.torque_2_last)
			



if __name__ == '__main__':
	rospy.init_node('rotor_combined_controller')
	try:
		obj = rotor_combined_controller().run()
	except:
		pass




