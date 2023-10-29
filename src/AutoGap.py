#!/usr/bin/env python3

import rospy
import random
import numpy as np
import time

from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive



class test:
    def __init__(self):
        rospy.init_node('test')
        self.drive_pub = rospy.Publisher('/drive', AckermannDriveStamped)
        self.marker_pub = rospy.Publisher("/test_marker", Marker, queue_size = 1)
        #rospy.Subscriber("/test_marker", Marker, queue_size = 2)
        rospy.Subscriber('/scan', LaserScan, self.LaserScanCallback, queue_size = 1)

        self.marker = Marker()

        self.marker.header.frame_id = "laser"
        self.marker.header.stamp = rospy.Time.now()
        #marker.lself.ifetime=0

        # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
        self.marker.type = 2
        self.marker.id = 0

        # Set the scale of the marker
        self.marker.scale.x = 0.3
        self.marker.scale.y = 0.3
        self.marker.scale.z = 0.3

        # Set the color
        self.marker.color.r = 0.0
        self.marker.color.g = 1.0
        self.marker.color.b = 0.0
        self.marker.color.a = 1.0

        # Set the pose of the marker
        self.marker.pose.position.x = 0
        self.marker.pose.position.y = 0
        self.marker.pose.position.z = 0
        self.marker.pose.orientation.x = 0.0
        self.marker.pose.orientation.y = 0.0
        self.marker.pose.orientation.z = 0.0
        self.marker.pose.orientation.w = 1.0

        self.drive=AckermannDriveStamped()

        
    def traitement(self):
        t1=time.perf_counter()
        
        risky_ids =[]
        find=False
        j=0
        while find == False:
            maxi=0.0
            max_id=0
            sortie=False


            for i in range(240,840):
                if i not in risky_ids :
                    #print ('id=',i,'  range=',range[i])
                    if self.ranges[i]>maxi:
                        #print ('max')
                        max_id=i
                        maxi=self.ranges[i]
            print ('maxi=',maxi,' max_id=',max_id)

            # max_angle = self.angle_min + (max_id * self.angle_increment)
            # max_x = self.ranges[max_id] * np.cos(abs(max_angle))
            # max_y = self.ranges[max_id] * np.sin(abs(max_angle))
            # print ('maxi=',maxi,' max_id=',max_id)
            # print ('max_x=',max_x,'  max_y=',max_y)
            # self.marker.header.stamp = rospy.Time.now()
            # self.marker.pose.position.x = max_x
            # self.marker.pose.position.y = max_y
            # self.marker_pub.publish(self.marker)
            
            # # VERIFICATION DE la distance des obstacles
            # for i in range(len(data.ranges)):
            #     dist=0
            #     if sortie==False :
            #         if data.ranges[i]<2:
            #             if i != max_id:
            #                 dist=np.sin((i-max_id)*data.angle_increment) * data.ranges[i]
            #                 #print ('dist=',dist,' i=',i)
            #                 if abs(dist) < 0.5:
            #                     #print ('dist < 0.5')
            #                     #print ('risky_ids=',risky_ids)
            #                     risky_ids.append(max_id)
            #                     #maxi=0.0
            #                     sortie=True

            # VERIFICATION DE la distance des obstacles
            for i in range(240,840):
                dist=0
                if sortie==False :
                    if self.ranges[i]<2:
                        if i != max_id:
                            dist=np.sin((i-max_id)*self.angle_increment) * self.ranges[i]
                            print ('range=',self.ranges[i], 'dist=',dist,' i=',i)
                            if abs(dist) < 0.5 and dist!=0.0:
                                #print ('dist < 0.5')
                                #print ('risky_ids=',risky_ids)
                                self.ranges[max_id]=0.0
                                #maxi=0.0
                                sortie=True
                                print ('sortie=',sortie)
                        #time.sleep(0.05)




            if sortie == False or max_id==0:
                find=True
                print ('find=',find)

            j+=1
        #print ('range=',ranges)
        
            #print ('find=',find)
            #print ('maxi=',maxi,'  max_id=',max_id)
            #time.sleep (0.2)
        print ('j=',j)
        #print ('risky_ids=',risky_ids)
        #print ('maxi=',maxi,'  max_id=',max_id)

        if max_id!=0:
            max_angle = (max_id - (len(self.ranges) / 2)) * self.angle_increment
            #max_angle = self.angle_min + (max_id * self.angle_increment)
            max_x = self.ranges[max_id] * np.cos(max_angle)
            max_y = self.ranges[max_id] * np.sin(max_angle)
            print ('maxi=',maxi,' max_id=',max_id)
            print ('max_x=',max_x,'  max_y=',max_y)
            self.marker.header.stamp = rospy.Time.now()
            self.marker.pose.position.x = max_x
            self.marker.pose.position.y = max_y
            self.marker_pub.publish(self.marker)
            
            self.drive.header.stamp = rospy.Time.now()
            self.drive.drive.steering_angle=max_angle/2
            self.drive.drive.speed=1#maxi/(1+abs(max_angle))
            self.drive_pub.publish(self.drive)
        else:
            self.marker.header.stamp = rospy.Time.now()
            self.marker.pose.position.x = 0
            self.marker.pose.position.y = 0
            self.marker_pub.publish(self.marker)

            self.drive.header.stamp = rospy.Time.now()
            self.drive.drive.steering_angle=0
            self.drive.drive.speed=0
            self.drive_pub.publish(self.drive)


        tf=time.perf_counter()
        print ('duree=',(tf-t1))

    def LaserScanCallback(self,data):
            self.ranges = list(data.ranges)
            self.angle_increment= data.angle_increment
            self.angle_min=data.angle_min

            self.traitement()


if __name__ == '__main__':
    try:
        test=test() #our main function
    except rospy.ROSInterruptException:
        print ('interrupted !')
    rate = rospy.Rate(30) #hz

    while not rospy.is_shutdown():
        rate.sleep()

