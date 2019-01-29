#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
PI = 3.1415926535897

def rotate():
    #new node
    rospy.init_node('robot_input', anonymous=True)
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()

    #User Input
    print("Let's a-go")
    speed = input("Input Speed:")
    time = input("Input Time(s):")
    isClockwise = input("Clockwise? (True 1/False 0):")


    #Check direction
    if(isClockwise):
        vel_msg.angular.z = -abs(speed)
    else:
        vel_msg.angular.z = abs(speed)
    #zero everything else
    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0

    t0 = rospy.Time.now().to_sec()
    t1 = t0

    while(t1-t0 < time):
        velocity_publisher.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()


    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)
    rospy.spin()

if __name__ == '__main__':
    try:
        rotate()
    except rospy.ROSInterruptException: pass
