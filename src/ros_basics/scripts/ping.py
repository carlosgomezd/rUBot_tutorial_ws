#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def ping():
    rospy.init_node('ping_node', anonymous=True)
    pub = rospy.Publisher('ping', String, queue_size=10)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        msg_str = "Ping"
        rospy.loginfo(msg_str)
        pub.publish(msg_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        ping()
    except rospy.ROSInterruptException:
        pass
