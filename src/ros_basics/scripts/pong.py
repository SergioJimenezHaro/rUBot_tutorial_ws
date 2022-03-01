#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

Entrada = 'Ping'

def pong_response(msg):
	global counter
	global Entrada
	if (Entrada == msg.data):
		counter = 'Pong'
		new_msg = String()
		new_msg.data = 'Pong'
		pub.publish(new_msg)
		rospy.loginfo("%s", counter)
		
	else:
		counter = 'Fail'
		new_msg = String()
		new_msg.data = 'Fail'
		pub.publish(new_msg)
		rospy.loginfo("%s", counter)
		return
	
	
rospy.init_node('pong_node')
pub = rospy.Publisher("/respuesta", String, queue_size=10)
sub = rospy.Subscriber("/nombre", String, pong_response)
rospy.spin()
