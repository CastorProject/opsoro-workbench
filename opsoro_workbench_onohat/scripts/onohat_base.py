#!/usr/bin/env python

#import onohat hardware library 
from hardware import Hardware
from opsoro_workbench_onohat.srv import SetStatusLed
from opsoro_workbench_onohat.srv import Ping 
from opsoro_workbench_onohat.srv import Reset  
import onohat_interface

import rospy
import time



class OnohatRos(object):
	def __init__(self):
		#initialize node
		rospy.init_node("onohat_base_controller")
		#set rates
		self.rate = rospy.Rate(10) # 10hz
		#log node initialization
		rospy.loginfo("Onohat base controller started")		
		#reset board
		Hardware.reset()
		#wait until reset
		time.sleep(1)
		#check board connection
		self.check_board_connection()
		#launch services 
		self.set_services()
		#launch publishers and subscribers
		self.set_publishers()
	
	
	def set_services(self):
		rospy.loginfo("setting services")
		#1. statusled service		
		rospy.Service("set_status_led", SetStatusLed, self.status_led_server)
		#2  ping service
		rospy.Service("reset_onohat", Reset, self.reset_server)
		#3  reset service
		rospy.Service("ping_onohat", Ping, self.ping_server)
	
	
	def set_publishers(self):
		pass

	
	def check_board_connection(self):
		res = Hardware.ping()
		
		if res:
			rospy.loginfo("Connection succesfully established")
		
		else:	
			rospy.logerr("cannot connect to the onohat board, check connection")
			#finish node
			rospy.signal_shutdown("failed connecting to the onohat")
		
		return res



	"""service request handlers"""

	def status_led_server(self, req):
		#turn led on
		if req.value:
			Hardware.led_on()
		else:
			Hardware.led_off()		
		
		return True
	
	def ping_server(self, req):
		
		res = Hardware.ping()
		return res

	def reset_server(self, req):
		Hardware.reset()
		return True


	"""main loop node"""
	def main_loop(self):
		
		
		self.rate.sleep()


if __name__=="__main__":
	onohat = OnohatRos()
	#enter main loop
	while not (rospy.is_shutdown()):
		onohat.main_loop()

