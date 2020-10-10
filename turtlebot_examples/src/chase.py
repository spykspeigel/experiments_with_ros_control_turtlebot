#!/usr/bin/env python
import rospy
from geometry_msgs.msg  import Twist
from geometry_msgs.msg  import Pose as pose1
from turtlesim.msg import Pose as pose2
from math import pow,atan2,sqrt,sin,cos

MAX_ACCELERATION=0.2
MAX_DECELERATION=-0.2


class turtlebot():

    def __init__(self):
        #Creating our node,publisher and subscriber
        rospy.init_node('turtlebot_chase', anonymous=True)
        self.vel_pub = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/rt_real_pose', pose1, self.t_callback)
        self.pose_subscriber = rospy.Subscriber('/turtle2/pose', pose2, self.callback)
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', pose2, self.target_callback)

        self.target_pose=pose2()
        self.pose = pose2()
        self.goal_pose=pose1()
        self.rate = rospy.Rate(10)

    #Callback function implementing the pose value received
    def callback(self, data):
        self.pose = data

    def t_callback(self, data):
        self.goal_pose=data
        
    def target_callback(self,data):
        self.target_pose=data
    def get_distance(self, goal_x, goal_y):
        distance = sqrt(pow((goal_x - self.pose.x), 2) + pow((goal_y - self.pose.y), 2))
        return distance
    
    def chase_target(self):
        
        distance_tolerance = .8
        vel_msg = Twist()
        
        error=0
        error_i=0
        error_d=0
        errort_i=0
        errort_d=0
        errort=0
        error_pre=0
        errort_pre=0

        kp=1.7
        kd=0.0001
        ki=0.0001

        kp_t=4.5
        ki_t=0.0099
        kd_t=.000001
        #print(self.goal_pose.position.x)
        rospy.wait_for_message('/rt_real_pose',pose1)
        while (sqrt(pow((self.target_pose.x - self.pose.x), 2) + pow((self.target_pose.y - self.pose.y), 2))>= distance_tolerance):

            #Error terms for speed
            error=sqrt(pow((self.goal_pose.position.x - self.pose.x), 2) + pow((self.goal_pose.position.y - self.pose.y), 2))
            error_d=error-error_d
            error_i=error_i+error_pre

            #Error terms for angular speed
            errort=atan2(self.goal_pose.position.y - self.pose.y, self.goal_pose.position.x - self.pose.x) - self.pose.theta
            errort=atan2(sin(errort),cos(errort))
            errort_d=errort-errort_pre
            errort_i=errort+errort_i
            

            v_temp=1#(kp * error +kd*error_d +ki*error_i)
            w_temp=kp_t*errort +kd_t*errort_d +ki_t*errort_i

            if (v_temp-self.pose.linear_velocity)>MAX_ACCELERATION:
                v_temp=self.pose.linear_velocity+MAX_ACCELERATION

            #linear velocity in the x-axis:
            vel_msg.linear.x = v_temp
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            #angular velocity in the z-axis:
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = w_temp
            #Publishing our vel_msg
            self.vel_pub.publish(vel_msg)
            error_pre=error
            errort_pre=errort
            self.rate.sleep()

        #Stopping our robot after the movement is over
        v_f=0

        while (v_f-self.pose.linear_velocity)<=MAX_DECELERATION:
            v_temp=self.pose.linear_velocity+MAX_DECELERATION
        #    self.decelerate(v_temp,w_temp)
        #else:
            vel_msg.linear.x = v_temp
            vel_msg.angular.z =0
            self.vel_pub.publish(vel_msg)
            
        #vel_msg.linear.x = 0
        #vel_msg.angular.z =0
        #self.vel_pub.publish(vel_msg)
        rospy.spin()

if __name__ == '__main__':
    try:
        #Testing our function
        x = turtlebot()
        
        x.chase_target()

    except rospy.ROSInterruptException: pass
