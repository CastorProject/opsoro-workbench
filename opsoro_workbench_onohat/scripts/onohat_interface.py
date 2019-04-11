#!/usr/bin/env python

#import onohat hardware library 
from hardware import Hardware

from opsoro_workbench_onohat.srv import SetStatusLed
 

import rospy
import time


"""
-------------------General Control------------------

ping_service(empty)
	Hardware.ping()

reset_onohat(empty)
	Hardware.reset()

set_led(bool)
	Hardware.led_on()
	Hardware.led_off()

"""


class StatusLedServer(object):
	def __init__(self):
		self.service_name = "set_status_led"

	
	#def handle_request(req):
		
		
		
