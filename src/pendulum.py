#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import Float64
from std_msgs.msg import String

def pendulum_publisher(topic, length=0.4, mag=0.175):
    # /exoskeleton/**_position_controller/command
    node_name = topic[13:16]+"pendulum"
    is_left = 1 if topic[13]=='L'else -1
    rospy.init_node("pendulum_node")
    pub = rospy.Publisher(topic,Float64, queue_size=10)
    omega = math.sqrt(9.8/length)
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        t = rospy.get_time()
        val = mag*math.sin(omega*t*is_left)
        pub.publish(val)
        rate.sleep()
        

if __name__ == '__main__':
    try:
        pendulum_publisher('/exoskeleton/LU_position_controller/command')
    except rospy.ROSInterruptException:
        pass
