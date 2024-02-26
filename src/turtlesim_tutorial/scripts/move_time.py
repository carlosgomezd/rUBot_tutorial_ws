#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import time  # Import time module
from turtlesim.msg import Pose

robot_x = 0
robot_y = 0

def pose_callback(pose):
    global robot_x, robot_y
    rospy.loginfo("Robot X = %f",pose.x)
    rospy.loginfo("Robot Y = %f\n",pose.y)
    robot_x = pose.x
    robot_y = pose.y


def move_time(lin_vel, ang_vel, duration):
    global robot_x, robot_y
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/turtle1/pose',Pose, pose_callback)
    rate = rospy.Rate(10)  # 10hz
    vel = Twist()

    start_time = time.time()  # Capture the start time

    while not rospy.is_shutdown():
        current_time = time.time()  # Update the current time each loop
        elapsed_time = current_time - start_time
        if(elapsed_time > duration):  # Check if the duration has passed
            rospy.loginfo("Duration reached")
            break

        vel.linear.x = lin_vel
        vel.angular.z = ang_vel
        pub.publish(vel)
        rate.sleep()

    # Stop the turtle after the time duration
    vel.linear.x = 0
    vel.angular.z = 0
    pub.publish(vel)

if __name__ == '__main__':
    try:
        rospy.init_node('move_time', anonymous=False)
        v = rospy.get_param("~v")  # Linear velocity
        w = rospy.get_param("~w")  # Angular velocity
        t = rospy.get_param("~t")  # Duration in seconds
        move_time(v, w, t)
    except rospy.ROSInterruptException:
        pass
