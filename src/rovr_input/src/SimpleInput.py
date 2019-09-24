#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

#new node
rospy.init_node('robot_input', anonymous=True)
velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
vel_msg = Twist()
speed = None

vel_msg.linear.y = 0
vel_msg.linear.z = 0
vel_msg.angular.x = 0
vel_msg.angular.y = 0


def main():
    print("Let's fucking go")
    speed = input("Input Speed: ")
    while True:
        userIn = raw_input("Input a Direction: ")

        if userIn == "w":
            vel_msg.linear.x = speed
            vel_msg.angular.z = 0
            velocity_publisher.publish(vel_msg)
        elif userIn == "s":
            vel_msg.linear.x = -speed
            vel_msg.angular.z = 0
            velocity_publisher.publish(vel_msg)
        elif userIn == "a":
            vel_msg.linear.x = 0
            vel_msg.angular.z = speed+2
            velocity_publisher.publish(vel_msg)
        elif userIn == "d":
            vel_msg.linear.x = 0
            vel_msg.angular.z = -speed-2
            velocity_publisher.publish(vel_msg)
        elif userIn == "0":
            vel_msg.linear.x = 0
            vel_msg.angular.z = 0
            velocity_publisher.publish(vel_msg)

        #rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: pass
