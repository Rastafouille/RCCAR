#!/usr/bin/env python3



import rospy
from std_msgs.msg import Float32, Float64, String

from vesc_msgs.msg import VescStateStamped
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

speed_percent=Float32()
speed=String()
steer=Float32()

max_speed=rospy.get_param("/autogap_node/max_speed",3.0)
max_steer=rospy.get_param("/autogap_node/max_steer",0.36)
angle_coef=rospy.get_param("/autogap_node/angle_coef",1)

def DriveCallback(data):
	
	steer.data=data.drive.steering_angle/max_steer/angle_coef*100
	speed_percent.data=data.drive.speed/max_speed*100
	speed.data=str(int (data.drive.speed))+' RPM'
	steer_pub.publish(steer)
	speed_percent_pub.publish(speed_percent)
	speed_pub.publish(speed)

rospy.init_node('rviz_overlay_node')
speed_percent_pub = rospy.Publisher("rviz_speed_percent", Float32, queue_size=1)
speed_pub = rospy.Publisher("rviz_speed", String, queue_size=1)
steer_pub = rospy.Publisher("rviz_steer", Float32, queue_size=1)

rospy.Subscriber('/drive', AckermannDriveStamped,DriveCallback,queue_size=1)



r = rospy.Rate(30.0)
while not rospy.is_shutdown():
	r.sleep()
