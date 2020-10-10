#!/usr/bin/env python
import rospy
from geometry_msgs.msg  import Twist
from turtlesim.msg import Pose
from math import pow,atan2,sqrt,sin,cos
import numpy as np
import sys
MAX_ACCELERATION=0.2

MAX_DECELERATION=-0.2


class turtlebot():

    def __init__(self):
        #Creating our node,publisher and subscriber
        rospy.init_node('turtlebot_controller', anonymous=True)
        self.vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.callback)
        self.pose = Pose()
        self.rate = rospy.Rate(10)
        self.goal_array=np.array([])
    #Callback function implementing the pose value received
    def callback(self, data):
        self.pose = data

 
    def move2goal(self):
        goal_pose = Pose()
        #goal_pose.x = float(input("Set your x goal:"))
        #goal_pose.y = float(input("Set your y goal:"))
        distance_tolerance = 0.8#float(input("Set your tolerance:"))
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

        for i in range(len(self.goal_array)):
            goal_pose.x=self.goal_array[i][0]
            goal_pose.y=self.goal_array[i][1]
            while sqrt(pow((goal_pose.x - self.pose.x), 2) + pow((goal_pose.y - self.pose.y), 2)) >= distance_tolerance:

                #Error terms for speed
                error=sqrt(pow((goal_pose.x - self.pose.x), 2) + pow((goal_pose.y - self.pose.y), 2))
                error_d=error-error_d
                error_i=error_i+error_pre

                #Error terms for angular speed
                errort=atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x) - self.pose.theta
                errort=atan2(sin(errort),cos(errort))
                errort_d=errort-errort_pre
                errort_i=errort+errort_i


                v_temp=kp * error +kd*error_d +ki*error_i
                w_temp=kp_t*errort +kd_t*errort_d +ki_t*errort_i

                if (v_temp-self.pose.linear_velocity)>MAX_ACCELERATION:
                    v_temp=self.pose.linear_velocity+MAX_ACCELERATION


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



        ##Stop the robot
            v_f=0

            while (v_f-self.pose.linear_velocity)<=MAX_DECELERATION:
                v_temp=self.pose.linear_velocity+MAX_DECELERATION

                vel_msg.linear.x = v_temp
                vel_msg.angular.z =0
                self.vel_pub.publish(vel_msg)
                self.rate.sleep()
            vel_msg.linear.x=0
            vel_msg.angular.z=0
            self.vel_pub.publish(vel_msg)
        rospy.spin()

if __name__ == '__main__':
    try:
        #Testing our function

        args=rospy.myargv(argv=sys.argv)

        if(len(args))<2:
            print("ERROR")
            sys.exit()
        

        x_coord1=float(args[1])
        y_coord1=float(args[2])
        x_coord2=float(args[3])
        y_coord2=float(args[4])
        x_coord3=float(args[5])
        y_coord3=float(args[6])
        x_coord4=float(args[7])
        y_coord4=float(args[8])
        x_coord5=float(args[9])
        y_coord5=float(args[10])
        x_coord6=float(args[11])
        y_coord6=float(args[12])
        x_coord7=float(args[13])
        y_coord7=float(args[14])
        x_coord8=float(args[15])
        y_coord8=float(args[16])

        
        bot = turtlebot()
        bot.goal_array=np.vstack([[x_coord1,y_coord1],[x_coord2,y_coord2],[x_coord3,y_coord3],[x_coord4,y_coord4],[x_coord5,y_coord5],[x_coord6,y_coord6],[x_coord7,y_coord7],[x_coord8,y_coord8]])

        print(bot.goal_array)
        bot.move2goal()

    except rospy.ROSInterruptException: pass
