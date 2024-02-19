#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

pub = None

def callback(data):
    global pub
    if data.data == "Ping":
        response = "Pong"
    else:
        response = "Failed!"
    rospy.loginfo("Data: \"%s\"", response)
    pub.publish(response)

def pong():
    global pub
    rospy.init_node('pong_node', anonymous=True)
    pub = rospy.Publisher('pong', String, queue_size=10)
    rospy.Subscriber('ping', String, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        pong()
    except rospy.ROSInterruptException:
        pass
