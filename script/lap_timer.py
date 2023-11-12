#!/usr/bin/env python3

import math
from math import sin, cos, pi
import numpy as np

import rospy
import tf
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3,PoseWithCovarianceStamped

class lap_timer:
    def __init__(self):

        rospy.init_node('lap_timer_node')
        #odom_pub = rospy.Publisher("odom", Odometry, queue_size=1)
        rospy.Subscriber('/odom', Odometry,self.OdomCallback,queue_size=1)
        rospy.Subscriber('/initialpose', PoseWithCovarianceStamped,self.InitialCallback,queue_size=1)
        self.text_pub = rospy.Publisher('/laps', String, queue_size = 1)
        self.InitTimer()

    def InitTimer(self):
        self.laps=[]
        self.start_time=rospy.Time.now()
        self.current_time=rospy.Time.from_sec(0)
        self.signe=0
        self.last_x=0
        self.text=String()
        self.text_laps=''
        self.text.data=self.text_laps
        self.text_pub.publish(self.text)

    def InitialCallback(self,data):
        self.InitTimer()
        return
        

    def OdomCallback(self,data):
        if self.last_x==0:
            self.last_x=data.pose.pose.position.x
        elif np.sign(data.pose.pose.position.x)== 1 and np.sign(self.last_x)==-1:
            lap_time=rospy.Time.now()-self.start_time
            self.laps.append(lap_time.to_sec())
            self.start_time=rospy.Time.now()
            self.current_time=rospy.Time.from_sec(0)
            self.last_x=data.pose.pose.position.x
            self.text_laps+='LAP '+str(len(self.laps))+' : '+str(round(lap_time.to_sec(),2))+ ' s  '
            self.text.data=self.text_laps
            self.text_pub.publish(self.text)
            #print ('LAPS ',self.laps)
        else:
            self.current_time=rospy.Time.now()-self.start_time
            self.last_x=data.pose.pose.position.x
        



if __name__ == '__main__':
    try:
        lap_timer=lap_timer() #our main function
    except rospy.ROSInterruptException:
        print ('interrupted !')
    rate = rospy.Rate(5) #hz

    while not rospy.is_shutdown():
        rate.sleep()
