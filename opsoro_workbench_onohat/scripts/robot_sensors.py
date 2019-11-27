#!/usr/bin/env python

#general purpose libraries
import rospy
import time
import pygame
import datetime
#Msg Library
from geometry_msgs.msg import Point
from std_msgs.msg import Int16
from std_msgs.msg import String
from std_msgs.msg import Bool


class robotsSensorsNode(object):
	def __init__(self, name):
		self.name = name
		rospy.init_node(self.name)
		self.rate = rospy.Rate(0.5)
		self.initVariables()
		self.initSubscribers()
		self.initPublishers()
		return

	def initSubscribers(self):
		self.leftLeg = rospy.Subscriber("/touchSensor/leftLeg", Int16, self.callbackLeftLegSensor)
		self.rightLeg = rospy.Subscriber("/touchSensor/rightLeg", Int16, self.callbackrightLegSensor)
		self.leftArm = rospy.Subscriber("/touchSensor/leftArm", Int16, self.callbackleftArmSensor)
		self.rightArm = rospy.Subscriber("/touchSensor/rightArm", Int16, self.callbackrightArmSensor)
		self.head = rospy.Subscriber("/touchSensor/head", Int16, self.callbackheadSensor)
		return

	def initVariables(self):
		self.rate = rospy.Rate(20) #10hz
		self.data = Point()
		self.emotion = String()
		self.leftLegValue = 0
		self.rightLegValue = 0
		self.leftArmValue = 0
		self.rightArmValue = 0
		self.headValue = 0
		self.refLL = 0
		self.refRL = 0
		self.refLA = 0
		self.refRA = 0
		self.refH = 0
		self.flag = 0
		return

	def initPublishers(self):
		self.Eyes_Pub  = rospy.Publisher("/eyesPosition", Point, queue_size= 10)
		self.emotionPub = rospy.Publisher('/emotions', String, queue_size = 10)
		return

	def Move_eyes(self, x, y):
		self.data.x = x
		self.data.y = y
		self.data.z = 0
		self.Eyes_Pub.publish(self.data)
		self.rate.sleep()

	def callbackLeftLegSensor(self, msg):
		self.leftLegValue = msg.data
		return

	def callbackrightLegSensor(self, msg):
		self.rightLegValue = msg.data
		return

	def callbackleftArmSensor(self, msg):
		self.leftArmValue = msg.data
		return

	def callbackrightArmSensor(self, msg):
		self.rightArmValue = msg.data
		return

	def callbackheadSensor(self, msg):
		self.headValue = msg.data
		return

	#Main
	def main(self):
		rospy.loginfo("[%s] robot_sensors node started ok", self.name)
		self.refLL = self.leftLegValue
		self.refRL = self.rightLegValue
		self.refLA = self.leftArmValue
		self.refRA = self.rightArmValue
		self.refH = self.headValue
		while not (rospy.is_shutdown()):
			self.rate.sleep()
			if self.refLL - self.leftLegValue > 30:
				if self.refLL - self.leftLegValue > 200:
					self.refLL = self.leftLegValue
				else:
					self.Move_eyes(-10,-20)
					#while self.refLL - self.leftLegValue > 20:
					#	continue
					#try:
					#	if pygame.mixer.get_init() != None:
					#		pygame.mixer.quit()
					#		pygame.mixer.init()
					#	else:
					#		pygame.mixer.init()
					#	pygame.mixer.music.load("/home/pi/Documents/pie.mp3")
					#	pygame.mixer.music.play()
					#	while pygame.mixer.music.get_busy() == True:
			    		#		continue
					#	pygame.mixer.quit()
					#except:
					#	print "speaker unavailable"
					now = datetime.datetime.now()
					currentDateTime = now.strftime("%Y-%m-%d %H:%M:%S")
					with open('/home/pi/logs/LL.log','a') as the_file:
						the_file.write(currentDateTime + "\n")
					#self.emotion.data = "neutral"
        				#self.emotionPub.publish(self.emotion)

			if self.refRL - self.rightLegValue > 30:
				if self.refRL - self.rightLegValue > 200:
					self.refRL = self.rightLegValue
				else:
					self.Move_eyes(10,-20)
					#while self.refRL - self.rightLegValue > 20:
					#	continue
					#try:
					#	if pygame.mixer.get_init() != None:
					#		pygame.mixer.quit()
					#		pygame.mixer.init()
					#	else:
					#		pygame.mixer.init()
					#	pygame.mixer.music.load("/home/pi/Documents/pie.mp3")
					#	pygame.mixer.music.play()
					#	while pygame.mixer.music.get_busy() == True:
			    		#		continue
					#	pygame.mixer.quit()
					#except:
					#	print "speaker unavailable"
                                        now = datetime.datetime.now()
                                        currentDateTime = now.strftime("%Y-%m-%d %H:%M:%S")
                                        with open('/home/pi/logs/RL.log','a') as the_file:
                                                the_file.write(currentDateTime + "\n")
					#self.emotion.data = "neutral"
                			#self.emotionPub.publish(self.emotion)

			if self.refLA - self.leftArmValue > 30:
				if self.refLA - self.leftArmValue > 200:
					self.refLA = self.leftArmValue
				else:
					self.Move_eyes(-20,-10)
					#while self.refLA - self.leftArmValue > 20:
					#	continue
					#try:
					#	if pygame.mixer.get_init() != None:
					#		pygame.mixer.quit()
					#		pygame.mixer.init()
					#	else:
					#		pygame.mixer.init()
					#	pygame.mixer.music.load("/home/pi/Documents/mano.mp3")
					#	pygame.mixer.music.play()
					#	while pygame.mixer.music.get_busy() == True:
				    	#		continue
					#	pygame.mixer.quit()
					#except:
					#	print "speaker unavailable"
                                	now = datetime.datetime.now()
                                        currentDateTime = now.strftime("%Y-%m-%d %H:%M:%S")
                                        with open('/home/pi/logs/LA.log','a') as the_file:
                                                the_file.write(currentDateTime + "\n")
					#self.emotion.data = "neutral"
                			#self.emotionPub.publish(self.emotion)

			if self.refRA - self.rightArmValue > 30:
				if self.refRA - self.rightArmValue > 200:
					self.refRA = self.rightArmValue
				else:
					self.Move_eyes(20,-10)
					#while self.refRA - self.rightArmValue > 20:
					#	continue
					#try:
					#	if pygame.mixer.get_init() != None:
					#		pygame.mixer.quit()
					#		pygame.mixer.init()
					#	else:
					#		pygame.mixer.init()
					#	pygame.mixer.music.load("/home/pi/Documents/mano.mp3")
					#	pygame.mixer.music.play()
					#	while pygame.mixer.music.get_busy() == True:
				    	#		continue
					#	pygame.mixer.quit()
					#except:
					#	print "speaker unavailable"
                                        now = datetime.datetime.now()
                                        currentDateTime = now.strftime("%Y-%m-%d %H:%M:%S")
                                        with open('/home/pi/logs/RA.log','a') as the_file:
                                                the_file.write(currentDateTime + "\n")
					#self.emotion.data = "neutral"
                			#self.emotionPub.publish(self.emotion)

			if self.refH - self.headValue > 30:
				if self.refH - self.headValue > 200:
					self.refH = self.headValue
				else:
					self.Move_eyes(0,30)
					#while self.refH - self.headValue > 20:
					#	continue
					#try:
					#	if pygame.mixer.get_init() != None:
					#		pygame.mixer.quit()
					#		pygame.mixer.init()
					#	else:
					#		pygame.mixer.init()
					#	pygame.mixer.music.load("/home/pi/Documents/antena.mp3")
					#	pygame.mixer.music.play()
					#	while pygame.mixer.music.get_busy() == True:
				    	#		continue
					#	pygame.mixer.quit()
					#except:
					#	print "speaker unavailable"
                                        now = datetime.datetime.now()
                                        currentDateTime = now.strftime("%Y-%m-%d %H:%M:%S")
                                        with open('/home/pi/logs/H.log','a') as the_file:
                                                the_file.write(currentDateTime + "\n")
					#self.emotion.data = "neutral"
                			#self.emotionPub.publish(self.emotion)
		return


if __name__=='__main__':
	robotsSensors = robotsSensorsNode("robotSensors")
	time.sleep(3)
	robotsSensors.main()
