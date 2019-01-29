#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

def move():
    #new node
    rospy.init_node('robot_input', anonymous=True)
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()

    #User Input
    print("Let's a-go")
    speed = input("Input Speed:")
    distance = input("Input Distance:")
    isForward = input("Forward? (True/False):")

    #Check direction
    if(isForward):
        vel_msg.linear.x = abs(speed)
    else:
        vel_msg.linear.x = -abs(speed)
    #zero everything else
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0


    #init time
    t0 = rospy.Time.now().to_sec()
    current_dis = 0

    #loop to move
    while(current_dis < distance):
        #Pub Velocity
        velocity_publisher.publish(vel_msg)
        #movement math
        t1=rospy.Time.now().to_sec()
        current_dis=speed*(t1-t0)
    #after done moving
    vel_msg.linear.x = 0
    velocity_publisher.publish(vel_msg)


    rospy.spin()

if __name__ == '__main__':
    try:
        move()
    except rospy.ROSInterruptException: pass
