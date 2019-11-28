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
                x = [i-30 for i in range(60)]
                y = [i-25 for i in range(50)]
                cont = 0
                cont2 = 0
		direction_x = 0
		direction_y = 0
                while not (rospy.is_shutdown()):
                        self.data.x = x[cont]
                        self.data.y = y[cont2]
                        if direction_x == 1:
                                cont -= 1
                        else:
                                cont += 1

                        if direction_y == 1:
                                cont2 -= 1
                        else:
                                cont2 += 1

                        time.sleep(0.1)

                        if(cont == len(x)-2):
                                direction_x = 1
                        elif(cont == 0):
                                direction_x = 0

                        if(cont2 == len(y)-2):
                                direction_y = 1
                        elif(cont2 == 0):
                                direction_y = 0

                        self.pub.publish(self.data)
                        self.rate.sleep()



if __name__=='__main__':

        E = EyesBehaviorNode()
        E.loop()
