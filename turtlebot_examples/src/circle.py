#!/usr/bin/env python
import rospy
from geometry_msgs.msg  import Twist
from geometry_msgs.msg  import Pose as pose1
from turtlesim.msg import Pose as pose2
from math import pow,atan2,sqrt,sin,cos
from numpy import random
MAX_ACCELERATION=0.2
MAX_DECELERATION=-0.2


class turtle_move_circle():

    def __init__(self):
        #Creating our node,publisher and subscriber
        rospy.init_node('turtlebot_controller', anonymous=True)
        self.vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.pose_sub = rospy.Subscriber('/turtle1/pose', pose2, self.callback)
        self.pose_pub =rospy.Publisher('/rt_real_pose',pose1,queue_size=10)
        self.pose_noisy_pub=rospy.Publisher('/rt_noisy_pose',pose1 ,queue_size=10)
        self.pose = pose2()
        self.rate = rospy.Rate(1.0/2.5)

    #Callback function implementing the pose value received
    def callback(self, data):
        self.pose = data

        

        #self.rate.sleep()
    def publish_data(self, event=None):

        p_t=pose1()
        p_t.position.x=self.pose.x
        p_t.position.y=self.pose.y
        self.pose_pub.publish(p_t)
        #random.normal(0,sd_trans*sd_trans)
        p_t_noisy=pose1()
        
        p_t_noisy.position.x=p_t.position.x+random.normal(0,.5)
        p_t_noisy.position.y=p_t.position.y+random.normal(0,.5)
        self.pose_noisy_pub.publish(p_t_noisy)

    
    def move_on_circle(self):
        vel_msg = Twist()
        vel_msg.linear.x = float(input("Set the desired speed:   "))
        radius= float(input("Set the desired radius:   "))
        vel_msg.angular.z=vel_msg.linear.x/radius
        rospy.Timer(rospy.Duration(2.5),self.publish_data)

        while not rospy.is_shutdown():
            self.vel_pub.publish(vel_msg)

        rospy.spin()

if __name__ == '__main__':
    try:
        #Testing our function
        x = turtle_move_circle()
        x.move_on_circle()

    except rospy.ROSInterruptException: pass
