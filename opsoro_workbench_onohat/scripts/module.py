import rospy
import time

import actuator.Actuator as Actuator

#base class for modules
class Module(object):

	def __init__(self, module_type = "dof", ):
		self.type = module_type  
		
		self.dof = Actuator()









class Sensor(object):
	def __init__(self):
		self.type = "capacitive"

		self.value = ""
