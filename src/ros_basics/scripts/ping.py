#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

rospy.init_node("ping_node", anonymous=True)
pub = rospy.Publisher("/nombre", String, queue_size=10)
rate = rospy.Rate(1)

while not rospy.is_shutdown():
	msg = String()
	msg.data = 'Pink'
	pub.publish(msg)
	rate.sleep()
