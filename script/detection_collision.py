#!/usr/bin/env python3

import rospy
import time
import numpy as np


from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

from std_msgs.msg import Bool, String

class detection_collision:
    def __init__(self):
        rospy.init_node('Collision_node')
        self.collision_pub = rospy.Publisher('/collision', Bool,queue_size=1)
        rospy.Subscriber('/odom', Odometry,self.OdomCallback,queue_size=1)
        rospy.Subscriber('/scan', LaserScan,self.LaserCallback,queue_size=1)
        self.text_pub = rospy.Publisher('/collision_text', String, queue_size = 1)

        self.collision_threshold=0.01
        self.collision = Bool()
        self.drive_speed=0
        self.min_range= 1000

        self.text=String()
        self.text.data=''
        self.text_pub.publish(self.text)


    def LaserCallback(self,data):
        self.min_range=np.min(data.ranges)


    def OdomCallback(self,data):
        if data.twist.twist.linear.x < self.collision_threshold and data.twist.twist.angular.z <self.collision_threshold and  self.min_range<0.2 :
            self.collision.data=True
            #print(rospy.Time.now(),'--collision detected--',self.min_range,'m')
            self.text.data='COLLISION!!'

        else:
            self.collision.data=False
            self.text.data=''
        self.text_pub.publish(self.text)
        self.collision_pub.publish(self.collision)
        

if __name__ == '__main__':
    try:
        detection_collision=detection_collision() #our main function
    except rospy.ROSInterruptException:
        print ('interrupted !')
    rate = rospy.Rate(5) #hz

    while not rospy.is_shutdown():
        rate.sleep()

