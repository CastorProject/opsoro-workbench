#!/usr/bin/env python

import rospy
import time

from geometry_msgs.msg import Point
from std_msgs.msg import String

class eyesManagerNode(object):
	def __init__(self, name):
		self.name = name
		rospy.init_node(self.name)
		self.rate = rospy.Rate(10) # 10hz
		self.initSubscribers()
		self.initPublishers()
		self.initVariables()
		return

	def initSubscribers(self):
		self.subDefEyesBehavior = rospy.Subscriber("/enableDefaultEyes", String, self.callbackDefaultEyes)
		self.subEyesPosition = rospy.Subscriber("/eyesPosition", Point, self.callbackEyesPosition)

	def initPublishers(self):
		self.pub = rospy.Publisher("/move_eyes", Point, queue_size = 10)
		return

	def initVariables(self):
		self.enableDefaultEyesBehavior = True
		self.eyesPosition = Point()
		self.eyesPositionX = 0.0
		self.eyesPositionY = 0.0
		self.eyesPositionZ = 0.0
		return

	def callbackDefaultEyes(self, msg):
		self.enableDefaultEyesBehavior = True
		return

	def callbackEyesPosition(self, msg):
		self.eyesPositionX = msg.x
		self.eyesPositionY = msg.y
		self.eyesPositionZ = msg.z
		self.enableDefaultEyesBehavior = False
		return

	def main(self):
		rospy.loginfo("[%s] eyes manager node started ok", self.name)
		x = [i-30 for i in range(60)]
		y = [i-25 for i in range(50)]
		contX = 0
		contY = 0
		directionX = 0
		directionY = 0

		while not (rospy.is_shutdown()):
			if self.enableDefaultEyesBehavior:
				self.eyesPosition.x = x[contX]
				self.eyesPosition.y = y[contY]

				if directionX == 1:
					contX -= 1
				else:
					contX += 1
				if directionY == 1:
					contY -= 1
				else:
					contY += 1
				time.sleep(0.1)
				if(contX == len(x)-2):
					directionX = 1
				elif(contX == 0):
					directionX = 0
				if(contY == len(y)-2):
					directionY = 1
				elif(contY == 0):
					directionY = 0

				self.pub.publish(self.eyesPosition)
				self.rate.sleep()

			else:
				self.eyesPosition.x = self.eyesPositionX
				self.eyesPosition.y = self.eyesPositionY
				self.eyesPosition.z = self.eyesPositionZ
				self.pub.publish(self.eyesPosition)
				self.rate.sleep()

if __name__ == '__main__':
	eyesManager = eyesManagerNode("eyesManager")
	eyesManager.main()
