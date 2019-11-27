<<<<<<< HEAD
f = open("Conf.txt",'r')
=======
f = open("/home/pi/catkin_ws/src/opsoro-workbench/castor_eyes/src/Conf.txt",'r')
>>>>>>> origin
lines = f.readlines()
text=[]
for l in lines:
	print(l)
	l = l.replace("\n","")
	l = l.split(";")
	text.append(l)
<<<<<<< HEAD
	print(l)
	print("fin linea")
=======
	#print(l)
	#print("fin linea")
>>>>>>> origin
print(text)
f.close()
