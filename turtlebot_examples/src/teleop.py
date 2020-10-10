#!/usr/bin/env python
import rospy
from geometry_msgs.msg  import Twist




if __name__=="__main__":
	
	try:
		rospy.init_node('turtlebot_controller', anonymous=True)
		vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)	
		v_temp=1
		w_temp=1
		vel_msg=Twist()
		vel_msg.linear.y = 0
		vel_msg.linear.z = 0

		vel_msg.angular.x = 0
		vel_msg.angular.y = 0
		while (1):
			
			vel_direction=float(input("Set the direction: 1-forward, 2-left, 3-right ::: "))
			
			if vel_direction==1.0:
				vel_msg.linear.x = v_temp
				vel_msg.angular.z = 0
				
				

			elif vel_direction==2.0:
				vel_msg.linear.x = 0
				vel_msg.angular.z = -w_temp				
				#vel_pub.publish()


			elif vel_direction==3.0:
				vel_msg.linear.x = 0
				vel_msg.angular.z = +w_temp
				#vel_pub.publish()
			vel_pub.publish(vel_msg)

	except rospy.ROSInterruptException: pass

		#rospy.spin()

