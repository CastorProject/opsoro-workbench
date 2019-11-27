#!usr/bin/python
#!usr/local/lib/python2.7
import os
#os.system("source /opt/ros/kinetic/setup.bash")
import rosgraph
import time
while not  rosgraph.is_master_online():
	print "waiting for roscore"
	time.sleep(1)


print "roscore running detected"

package = 'castor_eyes'
executable = 'eyes.py'
node_name = 'eyes'

print "start running node"
os.system("rosrun castor_eyes eyes.py")
print "command sent"
os.system("rosrun castor_eyes eyes_manager.py")
time.sleep(10)
print "after 10 seconds"
