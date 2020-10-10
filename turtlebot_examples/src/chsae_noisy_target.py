#!/usr/bin/env python
import rospy
from geometry_msgs.msg  import Twist
from geometry_msgs.msg  import Pose as pose1
from turtlesim.msg import Pose as pose2
from math import pow,atan2,sqrt,sin,cos
import numpy as np

MAX_ACCELERATION=0.2
MAX_DECELERATION=-0.2


class turtlebot():

    def __init__(self):
        #Creating our node,publisher and subscriber
        rospy.init_node('turtlebot_chase', anonymous=True)
        self.vel_pub = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/rt_real_pose', pose1, self.target_callback)
        self.pose_subscriber = rospy.Subscriber('/turtle2/pose', pose2, self.callback)

        self.pose = pose2()
        self.goal_pose=pose1()
        self.rate = rospy.Rate(10)

    #Callback function implementing the pose value received
    def callback(self, data):
        self.pose = data

    def target_callback(self, data):
        
        self.goal_pose=data
        



    def get_distance(self, goal_x, goal_y):
        distance = sqrt(pow((goal_x - self.pose.x), 2) + pow((goal_y - self.pose.y), 2))
        return distance
    
    def estimate_circle(self,p1,p2,p3): 
        p1=np.array([p1.position.x,p1.position.y,1])
        a1=abs(sqrt(p1.position.x**2+p1.position.y**2))
        
        p2=np.array([p2.position.x,p2.position.y,1])
        a2=abs(sqrt(p2.position.x**2+p2.position.y**2))

        p3=np.array([p3.position.x,p3.position.y,1])
        a3=abs(sqrt(p3.position.x**2+p3.position.y**2))
        
        B=np.vstack([p1,p2,p3])
        d=np.vstack([a1,a2,a3])


        ## Use Linear least square to find the center and the radius

        x,y,r=np.linalg.lstsq(B, d, rcond=None)[0]
        x=x/2
        y=y/2
        r=sqrt(r+x**2+y**2)
        return np.array([x,y,r])

    def get_circle_intersections(self,x0,y0,r0,x1,y1,r1):

    #distance between the circle    
        d=sqrt((x1-x0)**2 + (y1-y0)**2)

    # non intersecting
        if d > r0 + r1 :
            return np.array([])
    # One circle within other
        if d < abs(r0-r1):
            return np.array([])
    # coincident circles
        if d == 0 and r0 == r1:
            return np.array([])

        a=(r0**2-r1**2+d**2)/(2*d)
        h=sqrt(r0**2-a**2)
        x2=x0+a*(x1-x0)/d   
        y2=y0+a*(y1-y0)/d   
        x3=x2+h*(y1-y0)/d     
        y3=y2-h*(x1-x0)/d 

        x4=x2-h*(y1-y0)/d
        y4=y2+h*(x1-x0)/d

        return np.array([x3, y3, x4, y4])

    def chase_target(self):
        print('hey')
        distance_tolerance = .8
        vel_msg = Twist()
        goal_pose = pose1()
        v=0.3


        error=0
        error_i=0
        error_d=0
        errort_i=0
        errort_d=0
        errort=0
        error_pre=0
        errort_pre=0

        kp=8.0
        kd=0.0
        ki=0.1

        kp_t=4.0
        ki_t=0.0
        kd_t=0.0
        #print(self.goal_pose.position.x)
        p1=rospy.wait_for_message('/rt_real_pose',pose1)

        print(p1)
        p2=rospy.wait_for_message('/rt_real_pose',pose1)


        print(p2)
        p3=rospy.wait_for_message('/rt_real_pose',pose1)





        for t in np.linspace(20,40,20):
            intersections=self.get_circle_intersections(p[0],p[1],p[2],self.pose.x,self.pose.y,t*v)
            if len(intersections)!=0:
                break

        print(intersections)
        first_intersection_angle=atan2(intersections[1]-self.pose.y,intersections[0]-self.pose.x)        
        second_intersection_angle=atan2(intersections[3]-self.pose.y,intersections[2]-self.pose.x)

        if abs(first_intersection_angle)<abs(second_intersection_angle):
            goal_pose.position.x=intersections[0]
            goal_pose.position.y=intersections[1]
        else:
            goal_pose.position.x=intersections[2]
            goal_pose.position.y=intersections[3]

        while sqrt(pow((goal_pose.position.x - self.pose.x), 2) + pow((goal_pose.position.y - self.pose.y), 2)) >= distance_tolerance:

            #Error terms for speed
            error=sqrt(pow((goal_pose.position.x - self.pose.x), 2) + pow((goal_pose.position.y - self.pose.y), 2))
            error_d=error-error_d
            error_i=error_i+error_pre

            #Error terms for angular speed
            errort=atan2(goal_pose.position.y - self.pose.y, goal_pose.position.x - self.pose.x) - self.pose.theta
            errort=atan2(sin(errort),cos(errort))
            errort_d=errort-errort_pre
            errort_i=errort+errort_i
            print("hello",error)

            v_temp=v#kp * error +kd*error_d +ki*error_i
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
        print('hello')
        x.chase_target()

    except rospy.ROSInterruptException: pass
