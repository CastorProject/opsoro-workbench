#!/usr/bin/env python
#general library import
import rospy
import time
# create base msg and services
from dynamixel_workbench_msgs.srv import JointCommand
from dynamixel_workbench_msgs.srv import TorqueEnable
from sensor_msgs.msg import JointState

class Actuator(object):
	'''
	Actuator Class: Base class to inherit the general actuation functionality
	'''
	def __init__(self):
		#basic class info
		self.info = {
					 "ros_label"   : "/actuator",
				     "controller"  : "/base_controller",
					 "actuation"   : "electrical",
					 "motion"	   : "rotatory",
				     "hardware"    : "generic",
				     "id" 		   : 0,
					 "component_id": 0
				    }

		#actuator operation range
		self.max_limit  = 360
		self.min_limit 	= 0
		self.origin = 0

		#state variable
		self.state = {"state" : 0}
		#command
		self.command  = 0

	
	def deploy_actuator(self):
		'''
		function to update custom actuator's info and deploy services and topics
		'''
		self.set_info(self.info)
		self.set_topics()
		self.set_services()


	def get_info(self):
		'''
		return actuator's basic info
		'''
		return self.info


	def get_state(self):
		'''
		return actuator's current state (joint_state topic) 
		'''
		self.state = self.joint_subscriber.data
		return self.state


	def set_topics(self):
		'''
		function to deploy all publishers and subscribers
		'''
		#(1) Joint_state subscriber
		self.joint_subscriber = DmxJointStatesSubscriber(info = self.info)
		

	def set_services(self):
		'''
		function to deploy service servers
		'''
		pass

	def set_actuation_range(self, min= 0, max = 300, origin = 0):
		'''
		function to set the actuation range
		'''
		self.max_limit = max
		self.min_limit = min
		self.origin = origin

	def set_id(self, id):
		'''
		set actuator's id
		'''
		self.info['id'] = id


	def set_info(self, info):
		'''
		function to update the actuator's info
		'''
		self.info = info
		self.root = self.info['controller']


	def validate_position(self, command):
		'''
		function to validate the possition command sent to the actuator
		'''
		if command['value'] in range(self.min_limit, self.max_limit):
			return True
		else:
			return False


	def set_position(self, command, threaded = False,):
		'''
		set actuator's position
		'''
		#validate that command is within the limits
		assert self.validate_position(command), "Wrong command"

		name = self.root + "/goal_position"
		#print"requesting servie " + name
		service = CommandService(service_name  = name)

		if threaded:
			service.service_request_threaded(id = self.info['id'], val = command['value'])
		else:
			service.service_request(id = self.info['id'], val= command['value'])

	def set_speed(self, command, threaded = False):
		'''
		set actuator's speed
		'''
		name = self.root + "/goal_speed"
		service = CommandService(service_name  = name)

	def set_stiffness(self, command, threaded = False):
		'''
		set actuator's stiffness
		'''
		name = self.root + "/torque_enable"
		#service



#HELPER CLASSES

class CommandService(object):
	'''
	Helper class to handle command service requests for the actuator
	'''
    def __init__(self, service_name = "/goal_position"):
        #get service name
        self.service_name = service_name


    def service_request_threaded(self, id, val):
    	'''
    	function to call the service in a separate thread
    	'''
        threading.Thread(target = self.service_request, args = (id,val)).start()

    def service_request(self, id, val):
    	'''
		function to perform the command service request for the actuator
    	'''
        rospy.wait_for_service(self.service_name)
        try:
            goal_position = rospy.ServiceProxy(self.service_name, JointCommand)
            res = goal_position(id = id, value = val)
            time.sleep (0.001)
            return (res.result)

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e


class DmxJointStatesSubscriber(object):
	'''
	Helper class to handle joint state subscription
	'''
    def __init__(self, info ={"ros_label":"joint"}):

		self.topic_name = info["controller"] + "/joint_states"
		#create subscriber
		self.joint_subscriber = rospy.Subscriber(self.topic_name, JointState, self.callback)
		print(self.topic_name)
		self.data = []



    def callback(self, joint):
		'''
		callback function to handle and format joint state date
		'''
		d = dict(zip(joint.name,joint.position))
		self.data = d
