#!/usr/bin/env python

import rospy
import time

from geometry_msgs.msg import Point

class EyesBehaviorNode(object):
	def __init__(self):
		rospy.init_node("eyes_default_behavior")
		self.rate = rospy.Rate(20) #10hz
		self.pub  = rospy.Publisher("move_eyes", Point, queue_size= 10)
		self.data = Point()
		self.data.x = -10
		self.data.y = 5
		self.data.z = 0
				
	

	def loop(self):
		x = [i-10 for i in range(20)]
		y = [i-5 for i in range(10)]
		cont = 0
		while not (rospy.is_shutdown()):
			
			self.data.x = x[cont]
			self.data.y = 10
			cont += 1
			if(cont == 20):
				cont= 0
			self.pub.publish(self.data)
			self.rate.sleep()



if __name__=='__main__':
	
	E = EyesBehaviorNode()
	
	E.loop()
