#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import Float64
from std_msgs.msg import String

def data(time):
    time = 3.7699*time
    return 11.5134-19.0447*math.cos(1*time)+1.6275*math.sin(1*time)+0.70615*math.cos(2*time)-4.236*math.sin(2*time)+0.51072*math.cos(3*time)+0.44309*math.sin(3*time)+0.21466*math.cos(4*time)-0.054426*math.sin(4*time)

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
        # val = mag*math.sin(omega*t*is_left)
        val = -data(t)*math.pi/180
        pub.publish(val)
        rate.sleep()
        

if __name__ == '__main__':
    try:
        pendulum_publisher('/exoskeleton/LU_position_controller/command')
    except rospy.ROSInterruptException:
        pass
