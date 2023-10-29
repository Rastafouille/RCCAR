#!/usr/bin/env python3


## nuage de pint pour traj avec couleur vitesse
## fenetre variable en fonction de la distance des objets




import rospy,tf
import random
import numpy as np
import time

from geometry_msgs.msg import Twist, Point32
from visualization_msgs.msg import Marker
from sensor_msgs.msg import LaserScan, PointCloud, ChannelFloat32
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive



class autogap:
    def __init__(self):
        rospy.init_node('AutoGap_node')
        self.drive_pub = rospy.Publisher('/drive', AckermannDriveStamped)
        self.marker_pub = rospy.Publisher("/test_marker", Marker, queue_size = 1)
        self.marker2_pub = rospy.Publisher("/collision_marker", Marker, queue_size = 1)

        #rospy.Subscriber("/test_marker", Marker, queue_size = 2)
        rospy.Subscriber('/scan', LaserScan, self.LaserScanCallback, queue_size = 1)
        self.listener = tf.TransformListener()

        self.MAX_SPEED=7
        self.DIST_CRITIQUE=5
        self.DIST_SECU=0.7
        self.MAX_STEER=1.57
        self.MIN_ID=212
        self.MAX_ID=812

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

        self.marker2 = Marker()

        self.marker2.header.frame_id = "laser"
        self.marker2.header.stamp = rospy.Time.now()
        #marker.lself.ifetime=0

        # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
        self.marker2.type = 2
        self.marker2.id = 0

        # Set the scale of the marker
        self.marker2.scale.x = 0.5
        self.marker2.scale.y = 0.5
        self.marker2.scale.z = 0.5

        # Set the color
        self.marker2.color.r = 0.0
        self.marker2.color.g = 0.0
        self.marker2.color.b = 1.0
        self.marker2.color.a = 1.0

        # Set the pose of the marker
        self.marker2.pose.position.x = 0
        self.marker2.pose.position.y = 0
        self.marker2.pose.position.z = 0
        self.marker2.pose.orientation.x = 0.0
        self.marker2.pose.orientation.y = 0.0
        self.marker2.pose.orientation.z = 0.0
        self.marker2.pose.orientation.w = 1.0







        self.drive=AckermannDriveStamped()
        self.pcl_pub = rospy.Publisher("/pcl", PointCloud, queue_size = 1)
        self.pcl_msg = PointCloud()
        self.pcl_msg.header.frame_id = "map"
        self.point=Point32()
        # ch_int = ChannelFloat32("intensity", [0x00])
        # ch_rgb = ChannelFloat32("rgb", [0xff])
        # r_channel = ChannelFloat32(name="r", values=[0xff])
        # g_channel = ChannelFloat32(name="g", values=[0xff])
        # b_channel = ChannelFloat32(name="b", values=[0xff])
        # self.pcl_msg.channels = (ch_int, ch_rgb, r_channel, g_channel, b_channel)


    def rgb(self,minimum, maximum, value):
        minimum, maximum = float(minimum), float(maximum)
        ratio = 2 * (value-minimum) / (maximum - minimum)
        b = int(max(0, 255*(1 - ratio)))
        r = int(max(0, 255*(ratio - 1)))
        g = 255 - b - r
        return r, g, b

    def AddPointTraj(self):
        
        self.pcl_msg.header.stamp = rospy.Time(0)
        try:
            (trans,rot) = self.listener.lookupTransform('map','laser', rospy.Time(0))
            #print ('trans,rot =',(trans,rot))
            self.point.x=trans[0]    
            self.point.y=trans[1] 
            self.point.z=0   
            self.pcl_msg.points.append(self.point)
            if len(self.pcl_msg.points)>100: 
                del (self.pcl_msg.points[0])
                #print ('taille pcl=',len(self.pcl_msg.points))
            self.pcl_pub.publish(self.pcl_msg)
        except (tf.LookupException, tf.ConnectivityException,tf.ExtrapolationException):
            pass


        # ch_int = ChannelFloat32("intensity", [0x00])
        # ch_rgb = ChannelFloat32("rgb", [0xff])
        # r_channel = ChannelFloat32(name="r", values=[0xff])
        # g_channel = ChannelFloat32(name="g", values=[0xff])
        # b_channel = ChannelFloat32(name="b", values=[0xff])
        # pcl_msg.channels = (ch_int, ch_rgb, r_channel, g_channel, b_channel)
        

    def CalculVitesse(self,angle,distance):
        if distance >=self.DIST_CRITIQUE:
            vitesse=self.MAX_SPEED*self.MAX_STEER/(self.MAX_STEER+abs(angle))
        else :
            vitesse=self.MAX_SPEED*np.square(distance/self.DIST_CRITIQUE)*self.MAX_STEER/(self.MAX_STEER+abs(angle))

        #print('vitesse=',vitesse)
        return vitesse
        
    def traitement(self):
        t1=time.perf_counter()
        maxi=0.0
        max_id=0
        self.marker.color.r = 0.0
        self.marker.color.g = 1.0

        max_id=np.argmax(self.ranges[self.MIN_ID:self.MAX_ID])+self.MIN_ID
        maxi=self.ranges[max_id]
        # for i in range(240,840):
        #     if self.ranges[i]>maxi:
        #         max_id=i
        #         maxi=self.ranges[i]
        #print ('maxi=',maxi,' max_id=',max_id)


# VERIFICATION DE la distance des obstacles
        sortie=False
        mini=1000
        min_id=0
        dist=0
        for i in range(256,768):
            dist=0

            ##### A OPTIMISER AVEC NP
            if self.ranges[i]<3 and sortie==False:
                if i != max_id:
                    dist=np.sin((i-max_id)*self.angle_increment) * self.ranges[i]
                    if abs(dist) < self.DIST_SECU:
                        # Set the color
                        self.marker.color.r = 1.0
                        self.marker.color.g = 0.0
                        sortie=True
                        if self.ranges[i]<mini:
                            min_id=i
                            mini=self.ranges[i]

        #####trouver mieux que ca !!
        print ('maxi=',maxi,' max_id=',max_id)
        if min_id!=0:
            if min_id>max_id : max_id-=int(100-self.ranges[min_id]*10)
            else : max_id+=int(100-self.ranges[min_id]*10)
            print ('collision i=',min_id, 'new max_id',max_id)

            col_angle = self.angle_min + (min_id * self.angle_increment)
            col_x = self.ranges[min_id] * np.cos(col_angle)
            col_y = self.ranges[min_id] * np.sin(col_angle)
            self.marker2.header.stamp = rospy.Time.now()
            self.marker2.pose.position.x = col_x
            self.marker2.pose.position.y = col_y
            self.marker2_pub.publish(self.marker2)



        #max_angle = (max_id - (len(self.ranges) / 2)) * self.angle_increment
        max_angle = self.angle_min + (max_id * self.angle_increment)
        max_x = self.ranges[max_id] * np.cos(max_angle)
        max_y = self.ranges[max_id] * np.sin(max_angle)
        #print ('maxi=',maxi,' max_id=',max_id)
        #print ('max_x=',max_x,'  max_y=',max_y)
        self.marker.header.stamp = rospy.Time.now()
        self.marker.pose.position.x = max_x
        self.marker.pose.position.y = max_y
        self.marker_pub.publish(self.marker)







            
        self.drive.header.stamp = rospy.Time.now()
        self.drive.drive.steering_angle=max_angle
        self.drive.drive.speed=self.CalculVitesse(max_angle,self.ranges[max_id])
        self.drive_pub.publish(self.drive)

        self.AddPointTraj()
        
        tf=time.perf_counter()
        #print ('duree=',(tf-t1))

    def LaserScanCallback(self,data):
            self.ranges = list(data.ranges)
            self.angle_increment= data.angle_increment
            self.angle_min=data.angle_min
            self.traitement()




if __name__ == '__main__':
    try:
        autogap=autogap() #our main function
    except rospy.ROSInterruptException:
        print ('interrupted !')
    rate = rospy.Rate(30) #hz

    while not rospy.is_shutdown():
        rate.sleep()

