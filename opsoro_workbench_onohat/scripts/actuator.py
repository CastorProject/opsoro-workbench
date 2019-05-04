import rospy
from dynamixel_workbench_msgs.srv import JointCommand

from opsoro_workbench_onohat.srv import ServoCommand

#base actuator class (for servo, dc motor)
class Actuator(object):
	def __init__(self):
		#hardware resource
		self.info = {
					 "ros_label": "/actuator"
				     "type"		: "base actuator",
				     "hardware" : "generic",
				     "id" 		: 0
				    }
		#state
		self.max_limit  = 0
		self.min_limit 	= 360

		self.state = {"state" = 0}
		#command
		self.command  = 0

		#service definitions
		self.set_topics()
		self.set_services()

	def set_topics(self):
		pass
	def set_services(self):
		pass

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
		return self.state	
	
	def set_command(self, command, threaded = False):
		'''
		set actuator's command
		'''

		service = CommandService()

		if threaded:
			service.service_request_threaded(id = command['id'], val= command['value'])
		else:
			service.service_request(id = command['id'], val= command['value'])
		#call servide
		pass

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



class DmxServo(Actuator):
	"""docstring for DmxServo"""
	def __init__(self, arg):
		super(DmxServo, self).__init__()
		self.arg = arg

class OnohatServo(Actuator):
	"""docstring for OnohatServo"""
	def __init__(self, arg):
		super (OnohatServo, self).__init__()
		self.arg = arg
		
		
