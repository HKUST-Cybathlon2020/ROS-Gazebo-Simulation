#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import Float64
from std_msgs.msg import String

def data_LU(time):
    time = 3.7699*time
    return 11.5134-19.0447*math.cos(1*time)+1.6275*math.sin(1*time)+0.70615*math.cos(2*time)-4.236*math.sin(2*time)+0.51072*math.cos(3*time)+0.44309*math.sin(3*time)+0.21466*math.cos(4*time)-0.054426*math.sin(4*time)

def data_LD(time):
    time = 3.7699*time
    return -22.641+12.5562*math.cos(1*time)-19.7304*math.sin(1*time)+5.7732*math.cos(2*time)+6.0234*math.sin(2*time)-1.5175*math.cos(3*time)+0.51306*math.sin(3*time)-0.51899*math.cos(4*time)+0.19866*math.sin(4*time)
def data_RU(time):
    time = 3.7699*time
    return 14.1674+19.2834*math.cos(1*time)-1.6651*math.sin(1*time)-0.89819*math.cos(2*time)-4.6888*math.sin(2*time)-1.2296*math.cos(3*time)+0.3614*math.sin(3*time)+0.0083281*math.cos(4*time)+0.29098*math.sin(4*time)

def data_RD(time):
    time = 3.7699*time
    return -27.5046-8.1823*math.cos(1*time)+19.4181*math.sin(1*time)+8.0785*math.cos(2*time)+6.2189*math.sin(2*time)+1.7045*math.cos(3*time)-0.75302*math.sin(3*time)-0.23211*math.cos(4*time)-0.014863*math.sin(4*time)

def data_publisher():
    # /exoskeleton/**_position_controller/command
    rospy.init_node("data_publisher")
    pub_LU = rospy.Publisher('/exoskeleton/LU_position_controller/command',Float64, queue_size=10)
    pub_RU = rospy.Publisher('/exoskeleton/RU_position_controller/command',Float64, queue_size=10)
    pub_LD = rospy.Publisher('/exoskeleton/LD_position_controller/command',Float64, queue_size=10)
    pub_RD = rospy.Publisher('/exoskeleton/RD_position_controller/command',Float64, queue_size=10)
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        t = rospy.get_time()
        val_LU = -data_LU(t)*math.pi/180
        val_RU = -data_RU(t)*math.pi/180
        val_LD = -data_LD(t)*math.pi/180
        val_RD = -data_RD(t)*math.pi/180
        pub_LU.publish(val_LU)
        pub_RU.publish(val_RU)
        pub_LD.publish(val_LD)
        pub_RD.publish(val_RD)
        rate.sleep()
        

if __name__ == '__main__':
    try:
        data_publisher()
    except rospy.ROSInterruptException:
        pass
