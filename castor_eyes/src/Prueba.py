f = open("/home/pi/catkin_ws/src/opsoro-workbench/castor_eyes/src/Conf.txt",'r')
lines = f.readlines()
text=[]
for l in lines:
	print(l)
	l = l.replace("\n","")
	l = l.split(";")
	text.append(l)
	#print(l)
	#print("fin linea")
print(text)
f.close()
