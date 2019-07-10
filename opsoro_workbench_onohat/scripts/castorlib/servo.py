#!/usr/bin/env python

import rospy
import time

from actuator import Actuator

#custom control service and messages
#from dynamixel_workbench_msgs.srv import JointCommand
from opsoro_workbench_onohat.srv import ServoCommand
from opsoro_workbench_onohat.srv import EnablePcaPower
#from dynamixel_workbench_msgs.srv import TorqueEnable

print ServoCommand


class DmxServo(Actuator):
    '''
    Dynamixel servo class: Inherits the base Actuator class
    '''
    def __init__(self):
        super(DmxServo, self).__init__()
        #set custom dynamixel motor info
        self.info = {
					 "ros_label"   : "head",
				     "controller"  : "/dmx_controller",
					 "actuation"   : "electrical",
					 "motion"	   : "rotatory",
				     "hardware"    : "dynamixel",
				     "id" 		   : 0,
                     "component_id": 0
				    }
        #deploy actuator
        self.deploy_actuator()

    def set_motor_id(self, id):
        '''
        set dynamixel-motor id
        '''
        if id in range(1, 255):
            self.set_id(id)
            return True
        else:
            return False



class OnohatServo(Actuator):
    '''
    Onohat servo class: inherits the base Actuator class
    '''
    def __init__(self):
        super(OnohatServo, self).__init__()
        self.info = {
					 "ros_label"   : "hand",
				     "controller"  : "/onohat_controller",
					 "actuation"   : "electrical",
					 "motion"	   : "rotatory",
				     "hardware"    : "built-in pca",
				     "id" 		   : 0,
                     "component_id": 0
				    }

	self.set_srv_types(ServoCommand, EnablePcaPower)
	#deploy actuator
    	self.deploy_actuator()


    def set_motor_id(self, id):
        '''
        set pca servo id
        '''
        if id in range(1,15):
            self.set_id(id)
            return True
        else:
            False


if __name__ == '__main__':
    rospy.init_node("servo")

    rate = rospy.Rate(10) # 10hz

    servo  = DmxServo()
    shoulder = DmxServo()
    hand = OnohatServo()
    finger = OnohatServo()
	

    servo.set_motor_id(1)
    shoulder.set_motor_id(2)
    hand.set_motor_id(1)
    finger.set_motor_id(2)



    hand.set_actuation_range(min =800, origin =1000, max = 2000)
    #hand.set_stiffness(command = True)
    servo.set_actuation_range(min = 50, origin = 350, max = 650)
    shoulder.set_actuation_range(min =370,origin = 370,max= 800)
    finger.set_actuation_range(min = 900, origin =1500, max = 2600)

    print(hand.info)
    time.sleep(3)
    hand.set_position(command ={'value':801}, threaded = True)
    servo.set_position(command = {'value' : 51}, threaded = True)
    shoulder.set_position(command = {'value':371}, threaded = True)

    while not (rospy.is_shutdown()):
        print(hand.get_state())
	hand.set_position(command ={'value':1999}, threaded = True)
    	servo.set_position(command = {'value' : 649}, threaded = True)
    	shoulder.set_position(command = {'value':799}, threaded = True)
	time.sleep(4)
	finger.set_position(command = {'value':2500}, threaded = True)
	hand.set_position(command ={'value':801}, threaded = True)
    	servo.set_position(command = {'value' : 51}, threaded = True)
    	shoulder.set_position(command = {'value':371}, threaded = True)
	time.sleep(4)
	hand.set_position(command ={'value':1999}, threaded = True)
    	servo.set_position(command = {'value' : 649}, threaded = True)
    	shoulder.set_position(command = {'value':799}, threaded = True)
	finger.set_position(command = {'value':901}, threaded = True)
	time.sleep(4)
	hand.set_position(command ={'value':801}, threaded = True)
    	servo.set_position(command = {'value' : 51}, threaded = True)
    	shoulder.set_position(command = {'value':371}, threaded = True)
        #print servo.get_info()
	time.sleep(4)
        rate.sleep()
