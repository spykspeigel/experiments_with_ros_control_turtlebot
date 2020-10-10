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
        self.pose_subscriber = rospy.Subscriber('/rt_noisy_pose', pose1, self.t_callback)
        self.t_pose_subscriber = rospy.Subscriber('/turtle2/pose', pose2, self.callback)
        self.target_pose_subscriber = rospy.Subscriber('/turtle1/pose', pose2, self.target_callback)

        self.target_pose=pose2()
        self.pose = pose2()
        self.t_pose=pose1()
        self.rate = rospy.Rate(10)

    #Callback function implementing the pose value received
    def callback(self, data):
        self.pose = data

    def t_callback(self, data):
        
        self.t_pose=data
        
    def target_callback(self,data):
        self.target_pose=data


    def get_distance(self, goal_x, goal_y):
        distance = sqrt(pow((goal_x - self.pose.x), 2) + pow((goal_y - self.pose.y), 2))
        return distance
    
    def estimate_circle(self,B,d): 



        ## Use Linear least square to find the center and the radius


        temp=np.linalg.lstsq(B, d, rcond=None)[0]
        
        x=temp[0][0]/2
        y=temp[1][0]/2
        r=sqrt(temp[2][0]+x**2+y**2)
        c_r= np.array([x,y,r])
        return c_r

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

    def find_goal(self,candr,p0,v,target_v,tol):
        goal_pose=pose1()
        ind=0
        


                #finding the neareast point on the circle
        p0=np.array([candr[0],candr[1]])+candr[2]*(p0-np.array([candr[0],candr[1]]))/np.linalg.norm(p0-np.array([candr[0],candr[1]]))

        d=sqrt(pow(candr[0]-self.pose.x,2)+pow(candr[1]-self.pose.y,2))
        t_lower=(d-candr[2])/v
        t_higher=(d+candr[2])/v
        t_mean=(-t_lower+t_higher)/2
        theta0=atan2(-candr[1]+p0[1],-candr[0]+p0[0])
        for t in np.linspace(t_lower-t_mean/4,t_higher+t_mean/4,20):
            intersections=self.get_circle_intersections(candr[0],candr[1],candr[2],self.pose.x,self.pose.y,t*v)
            print(t)
            if len(intersections)!=0:
                x_target=candr[2]*sin((target_v/candr[2])*t+theta0)+p0[0]        
                y_target=-candr[2]*cos((target_v/candr[2])*t+theta0)+p0[1]
                first_intersection_distance=pow(intersections[1]-y_target,2)+pow(intersections[0]-x_target,2)
                
                second_intersection_distance=pow(intersections[3]-y_target,2)+pow(intersections[2]-x_target,2)
                if first_intersection_distance<tol:
                    goal_pose.position.x=intersections[0]
                    goal_pose.position.y=intersections[1]
                    ind=1
                elif second_intersection_distance<tol:
                    goal_pose.position.x=intersections[2]
                    goal_pose.position.y=intersections[3]                    
                    ind=1
            if ind==1:
                print('sucess')
                break
        if ind==0:
            goal_pose.position.x=p0[0]
            goal_pose.position.y=p0[1]
        return (goal_pose,ind)

    def chase_target(self):
        print('hey')
        distance_tolerance = .5
        vel_msg = Twist()
        goal_pose = pose1()
        v=0.3
        target_v=0.8

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
        p1=rospy.wait_for_message('/rt_noisy_pose',pose1)

        
        p2=rospy.wait_for_message('/rt_noisy_pose',pose1)


        
        p3=rospy.wait_for_message('/rt_noisy_pose',pose1)

        a1=(p1.position.x**2+p1.position.y**2)
        p1_t=np.array([p1.position.x,p1.position.y,1])
        
        a2=(p2.position.x**2+p2.position.y**2)
        p2_t=np.array([p2.position.x,p2.position.y,1])

        a3=(p3.position.x**2+p3.position.y**2)
        p3_t=np.array([p3.position.x,p3.position.y,1])
        
        B=np.vstack([p1_t,p2_t,p3_t])
        d=np.vstack([a1,a2,a3])



        c_r=self.estimate_circle(B,d)
        
        goal_pose,ind=self.find_goal(c_r,np.array([p3.position.x,p3.position.y]),v,target_v,distance_tolerance)
        
        ind_i=0
        while(sqrt(pow((self.target_pose.x - self.pose.x), 2) + pow((self.target_pose.y - self.pose.y), 2))>= .8): 
            
            
            p_temp=self.t_pose
           
            
            if p3.position.x != self.t_pose.position.x and ind==0 and ind_i==0:
                a_temp=(p_temp.position.x**2+p_temp.position.y**2)
                p_t=np.array([p_temp.position.x,p_temp.position.y,1])
                B=np.vstack([B,p_t])
                d=np.vstack([d,a_temp])
                c_r=self.estimate_circle(B,d)
                
                goal_pose,ind=self.find_goal(c_r,np.array([p_temp.position.x,p_temp.position.y]),v,target_v,distance_tolerance)
                
                p3=self.t_pose
                ind_i=1


            #Error terms for speed
            error=sqrt(pow((goal_pose.position.x - self.pose.x), 2) + pow((goal_pose.position.y - self.pose.y), 2))
            error_d=error-error_d
            error_i=error_i+error_pre

            #Error terms for angular speed
            errort=atan2(goal_pose.position.y - self.pose.y, goal_pose.position.x - self.pose.x) - self.pose.theta
            errort=atan2(sin(errort),cos(errort))
            errort_d=errort-errort_pre
            errort_i=errort+errort_i
            

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
            self.rate.sleep()
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
