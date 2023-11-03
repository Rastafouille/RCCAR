#!/usr/bin/env python3


import rospy,tf
import random
import numpy as np
import time

from geometry_msgs.msg import Twist, Point32
from visualization_msgs.msg import Marker
from sensor_msgs.msg import LaserScan, PointCloud, ChannelFloat32, Joy
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


class joy_to_ackermann:
    def __init__(self):
        rospy.init_node('joy_to_ackermann_node')
        self.drive_pub = rospy.Publisher('/ackermann_cmd', AckermannDriveStamped,queue_size = 1)
        rospy.Subscriber('/joy', Joy, self.JoyCallback, queue_size = 1)

        self.joy_speed_axis=rospy.get_param("/joy_to_ackermann/joy_speed_axis")
        self.joy_angle_axis=rospy.get_param("/joy_to_ackermann/joy_angle_axis")
        self.joy_max_speed=rospy.get_param("/joy_to_ackermann/joy_max_speed")
        self.joy_max_angle=rospy.get_param("/joy_to_ackermann/joy_max_angle")
        
        self.joy_button_idx=rospy.get_param("/joy_to_ackermann/joy_button_idx")
        
        self.drive=AckermannDriveStamped()


    def JoyCallback(self,data):
    	if  data.buttons[self.joy_button_idx]==1:
    		self.drive.header.stamp = rospy.Time.now()
    		self.drive.drive.steering_angle=data.axes[self.joy_angle_axis]*self.joy_max_angle
    		self.drive.drive.speed=data.axes[self.joy_speed_axis]*self.joy_max_speed
    		self.drive_pub.publish(self.drive)
    	else:
    		self.drive.header.stamp = rospy.Time.now()
    		self.drive.drive.steering_angle=0
    		self.drive.drive.speed=0
    		self.drive_pub.publish(self.drive)
    	



if __name__ == '__main__':
    try:
        joy_to_ackermann=joy_to_ackermann() 
        print('done')#our main function
    except rospy.ROSInterruptException:
        print ('interrupted !')
    rate = rospy.Rate(30) #hz

    while not rospy.is_shutdown():
        rate.sleep()

