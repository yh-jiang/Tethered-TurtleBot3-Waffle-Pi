#!/usr/bin/env python3
import numpy as np 
import time
import threading
import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from gazebo_msgs.msg import LinkStates
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
		self.rate		= rospy.get_param('~rate', 100.0)
		self.m	 		= 1.5
		self.J 			= 0.146
		self.r			= 0.033
		self.R			= 0.144
		self.l 		  	= 0.02
		self.d 			= 1
		self.J			= self.J+self.m*self.l**2
		
		# Control Inputs
		self.quaternion_1	= np.array([0.0,0.0,0.0,1.0])
		self.pos_ref_1 		= np.matrix([0.0,0.0]).transpose()
		self.vel_ref_1 		= np.matrix([0.0,0.0]).transpose()
		self.quaternion_2	= np.array([0.0,0.0,0.0,1.0])
		self.pos_ref_2 		= np.matrix([0.0,0.0]).transpose()
		self.vel_ref_2 		= np.matrix([0.0,0.0]).transpose()


		self.pos_mea_1    	= np.matrix([0.0,0.0,0.0]).transpose()
		self.vel_mea_1 		= np.matrix([0.0,0.0,0.0]).transpose()
		self.v_d_1			= np.matrix([0.0, 0.0]).transpose()
		self.v_d_old_1	    = np.matrix([0.0, 0.0]).transpose()
		self.v_d_dot_1 		= np.matrix([0.0, 0.0]).transpose()
		self.pos_mea_2    	= np.matrix([0.0,0.0,0.0]).transpose()
		self.vel_mea_2 		= np.matrix([0.0,0.0,0.0]).transpose()
		self.v_d_2			= np.matrix([0.0, 0.0]).transpose()
		self.v_d_old_2	    = np.matrix([0.0, 0.0]).transpose()
		self.v_d_dot_2 		= np.matrix([0.0, 0.0]).transpose()

		# Controller_outputs
		self.dt 			= 0.01
		self.torque_1		=Float64()
		self.torque_2		=Float64()
		self.torque_3		=Float64()
		self.torque_4		=Float64()
		self.step_num		=0
		#pubs
		self.joint_pb1	=rospy.Publisher('/tethered_turtle_sim/turtle1/wheel_left_joint_controller/command',Float64,queue_size=1)
		self.joint_pb2	=rospy.Publisher('/tethered_turtle_sim/turtle1/wheel_right_joint_controller/command',Float64,queue_size=1)
		self.joint_pb3	=rospy.Publisher('/tethered_turtle_sim/turtle2/wheel_left_joint_controller/command',Float64,queue_size=1)
		self.joint_pb4	=rospy.Publisher('/tethered_turtle_sim/turtle2/wheel_right_joint_controller/command',Float64,queue_size=1)
		#subs
	    #self.sub1	=rospy.Subscriber('/crazyjack/ground_truth/odometry',Odometry,self.odometry_process,queue_size=1)
		self.linkstate  =rospy.Subscriber('/gazebo/link_states',LinkStates,self.data_process_link,queue_size=1)
		self.traj_sub_1   =rospy.Subscriber('/tethered_turtle_sim/turtle1/traj_command_pos',Twist,self.update_setpoints_pos_1,queue_size=1)
		self.traj_sub_2   =rospy.Subscriber('/tethered_turtle_sim/turtle2/traj_command_pos',Twist,self.update_setpoints_pos_2,queue_size=1)
		#self.traj_sub2  =rospy.Subscriber('/turtlebot3_waffle_pi_sim/traj_command_vel',Twist,self.update_setpoints_vel,queue_size=1)


	def data_process_link(self,Msg):
		self.pos_mea_1[0,0]  = Msg.pose[13].position.x
		self.pos_mea_1[1,0]  = Msg.pose[13].position.y
		self.quaternion_1[0] = Msg.pose[13].orientation.x
		self.quaternion_1[1] = Msg.pose[13].orientation.y
		self.quaternion_1[2] = Msg.pose[13].orientation.z
		self.quaternion_1[3] = Msg.pose[13].orientation.w
		rpy=euler_from_quaternion(self.quaternion_1,axes='sxyz')
		self.pos_mea_1[2,0] =rpy[2]#np.arctan2(2 * (self.quaternion[0] * self.quaternion[3] + self.quaternion[1] * self.quaternion[2]),
		#(1 - 2 * (self.quaternion[2] * self.quaternion[2] + self.quaternion[3] * self.quaternion[3])))
		#print(self.quaternion_1)
		self.vel_mea_1[0,0]	  = Msg.twist[13].linear.x
		self.vel_mea_1[1,0]	  = Msg.twist[13].linear.y
		self.vel_mea_1[2,0]	  = Msg.twist[13].angular.z
		#print(self.vel_mea)

		self.pos_mea_2[0,0]  = Msg.pose[30].position.x
		self.pos_mea_2[1,0]  = Msg.pose[30].position.y
		self.quaternion_2[0] = Msg.pose[30].orientation.x
		self.quaternion_2[1] = Msg.pose[30].orientation.y
		self.quaternion_2[2] = Msg.pose[30].orientation.z
		self.quaternion_2[3] = Msg.pose[30].orientation.w
		rpy=euler_from_quaternion(self.quaternion_2,axes='sxyz')
		self.pos_mea_2[2,0] =rpy[2]#np.arctan2(2 * (self.quaternion[0] * self.quaternion[3] + self.quaternion[1] * self.quaternion[2]),
		#(1 - 2 * (self.quaternion[2] * self.quaternion[2] + self.quaternion[3] * self.quaternion[3])))
		#print(self.quaternion_1)
		self.vel_mea_2[0,0]	  = Msg.twist[30].linear.x
		self.vel_mea_2[1,0]	  = Msg.twist[30].linear.y
		self.vel_mea_2[2,0]	  = Msg.twist[30].angular.z
		#print(self.vel_mea)

	def update_setpoints_pos_1(self,trajMsg):
		self.pos_ref_1 = np.matrix([trajMsg.linear.x, trajMsg.linear.y]).transpose()     #np.asarray(pos_setpoint)
		self.vel_ref_1 = np.matrix([trajMsg.angular.x,trajMsg.angular.y]).transpose()
	#def update_setpoints_vel(self,trajMsg):


	def update_setpoints_pos_2(self,trajMsg):
		self.pos_ref_2 = np.matrix([trajMsg.linear.x, trajMsg.linear.y]).transpose()     #np.asarray(pos_setpoint)
		self.vel_ref_2 = np.matrix([trajMsg.angular.x,trajMsg.angular.y]).transpose()
	#def update_setpoints_vel(self,trajMsg):


	def pos_controller_step(self):
		lx=20.0
		ly=20.0
		kx=5.0
		ky=5.0

		err_x_1 = self.pos_ref_1[0,0] - self.pos_mea_1[0,0]
		err_y_1 = self.pos_ref_1[1,0] - self.pos_mea_1[1,0]
		phi_1   = self.pos_mea_1[2,0]
		xx_1	=self.vel_ref_1[0,0] +lx*np.tanh(kx/lx*err_x_1)
		yy_1	=self.vel_ref_1[1,0] +ly*np.tanh(ky/ly*err_y_1)
		K_1	=np.matrix([[np.cos(phi_1),np.sin(phi_1)],[-1/self.d*np.sin(phi_1), 1/self.d*np.cos(phi_1)]])

		S_1 = np.mat([[np.cos(phi_1), -self.d * np.sin(phi_1)], [np.sin(phi_1), self.d * np.cos(phi_1)]])
		K2_1 = np.linalg.pinv(S_1)
		#print(K_1,K2_1)
		self.v_d_1= K_1*np.mat([xx_1,yy_1]).transpose()
		if (self.step_num<1):
			self.v_d_dot_1=np.mat([0,0]).transpose()
			self.v_d_old_1=self.v_d_1
		else:
			self.v_d_dot_1 = (self.v_d_1 - self.v_d_old_1) / self.dt
		self.v_d_old_1=self.v_d_1
		# self.step_num=self.step_num+1
		self.v_d_dot_1[0,0] = np.clip(self.v_d_dot_1[0,0], -8, 8)
		self.v_d_dot_1[1,0] = np.clip(self.v_d_dot_1[1,0], -20, 20)
		print(1,self.pos_ref_1,self.pos_mea_1,self.vel_ref_1,self.v_d_dot_1)

		err_x_2 = self.pos_ref_2[0,0] - self.pos_mea_2[0,0]
		err_y_2 = self.pos_ref_2[1,0] - self.pos_mea_2[1,0]
		phi_2   = self.pos_mea_2[2,0]
		xx_2	=self.vel_ref_2[0,0] +lx*np.tanh(kx/lx*err_x_2)
		yy_2	=self.vel_ref_2[1,0] +ly*np.tanh(ky/ly*err_y_2)
		K_2	=np.matrix([[np.cos(phi_2),np.sin(phi_2)],[-1/self.d*np.sin(phi_2), 1/self.d*np.cos(phi_2)]])

		S_2 = np.mat([[np.cos(phi_2), -self.d * np.sin(phi_2)], [np.sin(phi_2), self.d * np.cos(phi_2)]])
		K2_2 = np.linalg.pinv(S_2)
		#print(K_2,K2_2)
		self.v_d_2= K_2*np.mat([xx_2,yy_2]).transpose()
		if (self.step_num<1):
			self.v_d_dot_2=np.mat([0,0]).transpose()
			self.v_d_old_2=self.v_d_2
		else:
			self.v_d_dot_2 = (self.v_d_2 - self.v_d_old_2) / self.dt
		self.v_d_old_2=self.v_d_2
		# self.step_num=self.step_num+1
		self.v_d_dot_2[0,0] = np.clip(self.v_d_dot_2[0,0], -8, 8)
		self.v_d_dot_2[1,0] = np.clip(self.v_d_dot_2[1,0], -20, 20)
		print(1,self.pos_ref_2,self.pos_mea_2,self.vel_ref_2,self.v_d_dot_2)


		

	def update_control(self):
		t_1=self.pos_mea_1[2,0]
		t_dot_1=self.vel_mea_1[2,0]
		S_1=np.mat([[np.cos(t_1), -self.l*np.sin(t_1)],[np.sin(t_1), self.l*np.cos(t_1)],[0,1]])
		S_dot_1=np.mat([[-np.sin(t_1)*t_dot_1, -self.l*np.cos(t_1)*t_dot_1],[np.cos(t_1)*t_dot_1, -self.l*np.sin(t_1)*t_dot_1],[0.0,0.0]])
		self.vr_1=np.linalg.pinv(S_1)*self.vel_mea_1

		M_1=np.mat([[self.m, 0.0, self.m*self.l*np.sin(t_1)],[0, self.m, -self.m*self.l*np.cos(t_1)],[self.m*self.l*np.sin(t_1), -self.m*self.l*np.cos(t_1),self.J]])
		Vm_1=np.mat([[0.0,0.0,self.m*self.l*t_dot_1*np.cos(t_1)],[0.0,0.0,self.m*self.l*t_dot_1*np.sin(t_1)],[0.0,0.0,0.0]])
		E_1=1/self.r*np.mat([[np.cos(t_1),np.cos(t_1)],[np.sin(t_1),np.sin(t_1)],[self.R,-self.R]])
		M_bar_1=S_1.transpose()*M_1*S_1
		V_bar_1=S_1.transpose()*M_1*S_dot_1+S_1.transpose()*Vm_1*S_1
		E_bar_1=S_1.transpose()*E_1
		self.tau_1=np.linalg.inv(E_bar_1)*(M_bar_1*self.v_d_dot_1+V_bar_1*self.v_d_1)
		K_1=np.matrix([[1.0,0.0],[0.0,0.5]])
		#e=self.v_d-self.vr
		uk_1=K_1*self.v_d_1-K_1*self.vr_1
		u_1=M_bar_1*(uk_1+self.v_d_dot_1)+V_bar_1*self.vr_1
		self.tau_1=np.linalg.inv(E_bar_1)*u_1
		self.tau_1=np.clip(self.tau_1,-8,8)
		print(self.v_d_1,self.vr_1,self.tau_1)


		t_2=self.pos_mea_2[2,0]
		t_dot_2=self.vel_mea_2[2,0]
		S_2=np.mat([[np.cos(t_2), -self.l*np.sin(t_2)],[np.sin(t_2), self.l*np.cos(t_2)],[0,1]])
		S_dot_2=np.mat([[-np.sin(t_2)*t_dot_2, -self.l*np.cos(t_2)*t_dot_2],[np.cos(t_2)*t_dot_2, -self.l*np.sin(t_2)*t_dot_2],[0.0,0.0]])
		self.vr_2=np.linalg.pinv(S_2)*self.vel_mea_2

		M_2=np.mat([[self.m, 0.0, self.m*self.l*np.sin(t_2)],[0, self.m, -self.m*self.l*np.cos(t_2)],[self.m*self.l*np.sin(t_2), -self.m*self.l*np.cos(t_2),self.J]])
		Vm_2=np.mat([[0.0,0.0,self.m*self.l*t_dot_2*np.cos(t_2)],[0.0,0.0,self.m*self.l*t_dot_2*np.sin(t_2)],[0.0,0.0,0.0]])
		E_2=1/self.r*np.mat([[np.cos(t_2),np.cos(t_2)],[np.sin(t_2),np.sin(t_2)],[self.R,-self.R]])
		M_bar_2=S_2.transpose()*M_2*S_2
		V_bar_2=S_2.transpose()*M_2*S_dot_2+S_2.transpose()*Vm_2*S_2
		E_bar_2=S_2.transpose()*E_2
		self.tau_2=np.linalg.inv(E_bar_2)*(M_bar_2*self.v_d_dot_2+V_bar_2*self.v_d_2)
		K_2=np.matrix([[1.0,0.0],[0.0,0.5]])
		#e=self.v_d-self.vr
		uk_2=K_2*self.v_d_2-K_2*self.vr_2
		u_2=M_bar_2*(uk_2+self.v_d_dot_2)+V_bar_2*self.vr_2
		self.tau_2=np.linalg.inv(E_bar_2)*u_2
		self.tau_2=np.clip(self.tau_2,-8,8)
		print(self.v_d_2,self.vr_2,self.tau_2)

	def publish_command(self):
		self.torque_1.data=self.tau_1[1,0]
		self.torque_2.data=self.tau_1[0,0]
		self.torque_3.data=self.tau_2[1,0]
		self.torque_4.data=self.tau_2[0,0]
		#self.thrust_3.data=-1*self.thrust[2]
		#self.thrust_4.data=-1*self.thrust[3]
		self.joint_pb1.publish(self.torque_1)
		self.joint_pb2.publish(self.torque_2)
		self.joint_pb3.publish(self.torque_3)
		self.joint_pb4.publish(self.torque_4)
		#self.thrust_pb3.publish(self.thrust_3)
		#self.thrust_pb4.publish(self.thrust_4)


	def step(self):
		self.dt=1/self.rate
		self.pos_controller_step()
		self.update_control()
		#print(self.thrust)
		#print(self.beta)
		#time.sleep(0.02)		
		self.publish_command()
		self.step_num=self.step_num+1

	def run(self):
		rate =rospy.Rate(self.rate)
		while not rospy.is_shutdown():
			
			self.step()	
			rate.sleep()



if __name__ == '__main__':
	rospy.init_node('rotor_combined_controller')
	try:
		obj = rotor_combined_controller().run()
	except:
		pass




