#!/usr/bin/env python3

import math
from math import sin, cos, pi

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

def OdomCallback(data):
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, data.pose.pose.orientation.z)
    odom_broadcaster.sendTransform((data.pose.pose.position.x, data.pose.pose.position.y, 0.),odom_quat,data.header.stamp,"base_link","odom")

rospy.init_node('odometry_publisher')
odom_pub = rospy.Publisher("odom", Odometry, queue_size=1)
rospy.Subscriber('/odom', Odometry,OdomCallback,queue_size=1)
odom_broadcaster = tf.TransformBroadcaster()





r = rospy.Rate(30.0)
while not rospy.is_shutdown():
        r.sleep()



