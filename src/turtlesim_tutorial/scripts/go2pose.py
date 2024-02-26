#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import atan2, pow, sqrt, pi

class TurtleBot:

    def __init__(self):
        rospy.init_node('move_turtle', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.update_pose)
        self.pose = Pose()
        self.rate = rospy.Rate(10)
        self.state = "position"  # Start with moving to position

    def update_pose(self, data):
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)

    def euclidean_distance(self, goal_pose):
        return sqrt(pow((goal_pose.x - self.pose.x), 2) + pow((goal_pose.y - self.pose.y), 2))

    def linear_vel(self, goal_pose, constant=1.5):
        return constant * self.euclidean_distance(goal_pose)

    def steering_angle(self, goal_pose):
        return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

    def angular_vel(self, goal_pose, constant=6):
        return constant * (self.steering_angle(goal_pose) - self.pose.theta)

    def move_to_goal(self, goal_pose, distance_tolerance=0.1):
        vel_msg = Twist()

        # Loop until the turtle is within a specified distance tolerance from the goal
        while self.euclidean_distance(goal_pose) >= distance_tolerance:
            vel_msg.linear.x = self.linear_vel(goal_pose)
            vel_msg.angular.z = self.angular_vel(goal_pose)
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()
        
        # Once the turtle reached the goal, stop
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)
        rospy.loginfo("Position goal reached")

    def normalize_angle(self, angle):
        # Normalize the angle to be within the range of -pi to pi
        while angle > pi:
            angle -= 2 * pi
        while angle < -pi:
            angle += 2 * pi
        return angle

    def adjust_orientation(self, goal_pose, angle_tolerance=0.01):
        vel_msg = Twist()
        angle_difference = self.normalize_angle(goal_pose.theta - self.pose.theta)

        # Loop until the difference is within a specified tolerance
        while abs(angle_difference) > angle_tolerance:
            vel_msg.angular.z = 4 * angle_difference
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()
            # Recalculate the orientation difference
            angle_difference = self.normalize_angle(goal_pose.theta - self.pose.theta)

        # Once the orientation is reached, stop
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)
        rospy.loginfo("Orientation goal reached")

if __name__ == '__main__':
    try:
        x = float(input("Set your x goal: "))
        y = float(input("Set your y goal: "))
        theta = float(input("Set your orientation goal [0~6.283] (rad): "))
        goal_pose = Pose()
        goal_pose.x = x
        goal_pose.y = y
        goal_pose.theta = theta
        turtlebot = TurtleBot()
        turtlebot.move_to_goal(goal_pose)
        turtlebot.adjust_orientation(goal_pose)
    except rospy.ROSInterruptException:
        pass