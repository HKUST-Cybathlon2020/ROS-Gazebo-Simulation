#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import Float64
from std_msgs.msg import String

def data(time):
    time = 3.7699*time
    return 14.1674+19.2834*math.cos(1*time)-1.6651*math.sin(1*time)-0.89819*math.cos(2*time)-4.6888*math.sin(2*time)-1.2296*math.cos(3*time)+0.3614*math.sin(3*time)+0.0083281*math.cos(4*time)+0.29098*math.sin(4*time)


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
        pendulum_publisher('/exoskeleton/RU_position_controller/command')
    except rospy.ROSInterruptException:
        pass
