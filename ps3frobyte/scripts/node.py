#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64
from matplotlib import pyplot as plt


i = 0
once = 1
x = 0
y = 0

def update_line(data):
	global once
	global x
	global y
	global i
	if once:
		fig=plt.figure()
		plt.axis([0,1000,-90,90])

		x=list()
		y=list()

		plt.ion()
		plt.show()
		once = 0
	x.append(i)
	y.append(data.data)
	plt.scatter(i,data.data)
	i+=1
	plt.draw()

if __name__ == '__main__':
    try:
	print "Node up\n"
	sub = rospy.Subscriber('/frobyte/error',Float64, update_line);
	rospy.init_node('talker')
	rospy.spin()
    except rospy.ROSInterruptException:
        pass

