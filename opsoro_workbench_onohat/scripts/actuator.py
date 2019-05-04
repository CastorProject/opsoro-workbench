#!/usr/bin/env python
import rospy
import time
# create base msg and services
from dynamixel_workbench_msgs.srv import JointCommand
from dynamixel_workbench_msgs.srv import TorqueEnable
from sensor_msgs.msg import JointState

#from opsoro_workbench_onohat.srv import ServoCommand

#base actuator class (for servo, dc motor)
class Actuator(object):
	def __init__(self):
		print("init actuator  class")
		'''
		actuator info:
			type: ['dmx_controller','onohat_controller']
		'''
		self.info = {
					 "ros_label"   : "/actuator",
				     "controller"  : "/base_controller",
					 "actuation"   : "electrical",
					 "motion"	   : "rotatory",
				     "hardware"    : "generic",
				     "id" 		   : 0,
					 "component_id": 0
				    }
		#set root for services and topics
		self.set_info(self.info)

		#state
		self.max_limit  = 0
		self.min_limit 	= 360

		self.state = {"state" : 0}
		#command
		self.command  = 0

		#service definitions
		#self.set_topics()
		#self.set_services()

	def set_topics(self):
		print("topics created")
		self.joint_subscriber = DmxJointStatesSubscriber(info = self.info)
		print(self.joint_subscriber.topic_name)

	def set_services(self):
		pass

	#actuation depending on type
	def set_actuation_range(self,min = 0, max = 300, origin = 0):
		self.max_limit = max
		self.min_limit = min
		self.origin = origin


	def validate_position(self, command):
		if command['value'] in range(self.min_limit, self.max_limit):
			print"YES"
			return True
		else:
			print"NOP"
			return False


	def set_id(self, id):
		'''
		set actuator's id
		'''
		self.info['id'] = id


	def set_info(self, info):
		self.info = info
		self.root = self.info['controller']


	def get_info(self):
		'''
		return actuator basic info
		'''
		return self.info


	def get_state(self):
		'''
		return actuator's current state
		'''
		#subscriber
		self.state = self.joint_subscriber.data
		return self.state

	def set_position(self, command, threaded = False,):
		'''
		set actuator's position
		'''
		#validate that command is within the limits
		assert self.validate_position(command), "Wrong command"

		name = self.root + "/goal_position"
		print"requesting servie " + name
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
    def __init__(self, service_name = "/goal_position"):
        #get service name
        self.service_name = service_name


    def service_request_threaded(self, id, val):
        threading.Thread(target = self.service_request, args = (id,val)).start()

    def service_request(self, id, val):
        rospy.wait_for_service(self.service_name)
        try:
            goal_position = rospy.ServiceProxy(self.service_name, JointCommand)
            res = goal_position(id = id, value = val)
            time.sleep (0.001)
            return (res.result)

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e


class DmxJointStatesSubscriber(object):
    def __init__(self, info ={"ros_label":"joint"}):

		self.topic_name = info["controller"] + "/joint_states"
		#create subscriber
		self.joint_subscriber = rospy.Subscriber(self.topic_name, JointState, self.callback)
		print(self.topic_name)
		self.data = []



    def callback(self, joint):
		#joint.velocity
		d = dict(zip(joint.name,joint.position))
		self.data = d
