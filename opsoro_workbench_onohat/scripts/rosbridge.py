import roslibpy
import time

class RobotClient(object):
	def __init__(self, host ='10.30.0.71', port = 9999):
		#set port and host address
		self.port = port
		self.host = host
		#create ros-bridge client
		self.ros_client = roslibpy.Ros(host = self.host, port = self.port)
		#services
		self.pca_service = roslibpy.Service(self.ros_client, '/enable_servo_pca','opsoro_workbench_onohat/EnablePcaPower')
		self.goal_position_service = roslibpy.Service(self.ros_client, '/goal_position','opsoro_workbench_onohat/ServoCommand')
		self.status_led_service = roslibpy.Service(self.ros_client, '/set_status_led','opsoro_workbench_onohat/SetStatusLed')



	def get_manifest(self):
		'''
		function to retrieve the robot's manifest (modules, services and topics)
		'''

		pass


	def connect_to_robot(self):
		'''
		function to connect to the ros-bridge server
		'''
		self.ros_client.run()


	def check_connection(self):
		'''
		function to check ros-bridge server
		return: connection status
		'''
		return self.ros_client.is_connected


	def dissconnect(self):
		'''
		function to close ros-bridge socket connection
		'''
		self.ros_client.close()


	def set_services(self):
		'''
		set required service clients
		'''


	def on_success(self, res):
		'''
		callback function for success service request
		'''
		print "success" + str(res)


	def on_failed(self, res):
		'''
		callback function for failed service request
		'''
		print("something went wrong:")
		print(res)


	#################################ONOHAT#####################################
	def enable_pca_service(self, val):
		'''
		function to call the enable_servo_pca service
		parameters: bool
		'''
		#self.pca_service = roslibpy.Service(self.ros_client, '/enable_servo_pca','opsoro_workbench_onohat/EnablePcaPower')
		print("calling service")
		request = roslibpy.ServiceRequest(values = {"value":val})
		self.pca_service.call(request,self.on_success, self.on_failed)


	def set_servo(self,channel,val):
		'''
		function to call the goal_position service
		parameters: channel, value
		'''
		#self.goal_position_service = roslibpy.Service(self.ros_client, '/goal_position','opsoro_workbench_onohat/ServoCommand')
		print("calling goal position service")
		request = roslibpy.ServiceRequest(values = {"channel": channel, "value": val})
		self.goal_position_service.call(request,self.on_success, self.on_failed)

	def set_status_led(self, val):
		'''
		function to call the set_status_led services
		parameters: value
		'''
		#self.status_led_service = roslibpy.Service(self.ros_client, '/set_status_led','opsoro_workbench_onohat/SetStatusLed')
		print("calling status led service")
		request = roslibpy.ServiceRequest(values = {"value": val})
		self.status_led_service.call(request,self.on_success, self.on_failed)


if __name__ == '__main__':
	r = RobotClient(host ='192.168.1.101', port = 9999)

	r.connect_to_robot()
	print "checking  ROS-BRIDGE connection"
	print r.check_connection()

	time.sleep(1)
	a = True
	for i in range(10):
		r.set_status_led(val= a)
		a = not a
		time.sleep(1)

	time.sleep(1)
	print "enable pca power"
	r.enable_pca_service(val = True)

	time.sleep(1)
	r.set_servo(channel = 0, val = 800)
	time.sleep(2)
	r.set_servo(channel = 0, val = 2000)
	time.sleep(2)
	r.set_servo(channel = 0, val = 800)
	time.sleep(2)
	r.set_servo(channel = 0, val = 2000)
	time.sleep(2)


	print "disable pca power"
	r.enable_pca_service(val = False)
	time.sleep(2)
	r.dissconnect()
