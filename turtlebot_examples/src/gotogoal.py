#!/usr/bin/env python
import rospy
from geometry_msgs.msg  import Twist
from turtlesim.msg import Pose
from math import pow,atan2,sqrt,sin,cos
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
        self.goal_pose=Pose()
    #Callback function implementing the pose value received
    def callback(self, data):
        self.pose = data

 
    def move2goal(self):
        distance_tolerance = 0.1
        vel_msg = Twist()
        
        error=0
        error_i=0
        error_d=0
        errort_i=0  
        errort_d=0
        errort=0

        error_pre=0
        errort_pre=0

        kp=1.0
        kd=0.0001
        ki=0.0001

        kp_t=14
        ki_t=0.000001
        kd_t=.00001
        while sqrt(pow((self.goal_pose.x - self.pose.x), 2) + pow((self.goal_pose.y - self.pose.y), 2)) >= distance_tolerance:

            #Error terms for speed
            error=sqrt(pow((self.goal_pose.x - self.pose.x), 2) + pow((self.goal_pose.y - self.pose.y), 2))
            error_d=error-error_d
            error_i=error_i+error_pre

            #Error terms for angular speed
            errort=atan2(self.goal_pose.y - self.pose.y, self.goal_pose.x - self.pose.x) - self.pose.theta
            errort=atan2(sin(errort),cos(errort))
            errort_d=errort-errort_pre
            errort_i=errort+errort_i


            v_temp=kp * error +kd*error_d +ki*error_i
            w_temp=kp_t*errort +kd_t*errort_d +ki_t*errort_i

            #if (v_temp-self.pose.linear_velocity)>MAX_ACCELERATION:
            #    v_temp=self.pose.linear_velocity+MAX_ACCELERATION


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
        print('STOP')
        #while (v_f-self.pose.linear_velocity)<=MAX_DECELERATION:
        #    v_temp=self.pose.linear_velocity+MAX_DECELERATION

        #    vel_msg.linear.x = v_temp
        #    vel_msg.angular.z =0
        #    self.vel_pub.publish(vel_msg)
        #    self.rate.sleep()
        rospy.spin()

if __name__ == '__main__':
    try:
        #Testing our function

        args=rospy.myargv(argv=sys.argv)

        if(len(args))<2:
            print("ERROR")
            sys.exit()
        
        x_coord=float(args[1])
        y_coord=float(args[2])

        bot = turtlebot()
        bot.goal_pose.x=x_coord
        bot.goal_pose.y=y_coord
        print(bot.goal_pose)
        bot.move2goal()

    except rospy.ROSInterruptException: pass
