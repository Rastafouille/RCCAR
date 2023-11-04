#!/usr/bin/env python3



import rospy
from std_msgs.msg import Float32, Float64, String

from vesc_msgs.msg import VescStateStamped

speed_percent=Float32()
speed=String()
steer=Float32()

joy_max_speed=rospy.get_param("/joy_to_ackermann/joy_max_speed",3.0)
speed_to_erpm_gain=rospy.get_param("/speed_to_erpm_gain")
speed_to_erpm_offset=rospy.get_param("/speed_to_erpm_offset")
steering_angle_to_servo_gain=rospy.get_param("/steering_angle_to_servo_gain")
steering_angle_to_servo_offset=rospy.get_param("/steering_angle_to_servo_offset")


def SteerCallback(data):
	steer.data=(data.data-steering_angle_to_servo_offset)/0.3*(-100)
	steer_pub.publish(steer)

def SpeedCallback(data):
	# if data.data>=0:
	# 	speed_percent.data=(data.data-speed_to_erpm_offset)/(speed_to_erpm_gain*joy_max_speed)*100
	# else:
	speed_percent.data=data.state.speed/(speed_to_erpm_gain*joy_max_speed)*100
	
	speed_percent_pub.publish(speed_percent)
	 
	speed.data=str(int (data.data))+' RPM'
	speed_pub.publish(speed)



rospy.init_node('rviz_overlay_node')
speed_percent_pub = rospy.Publisher("rviz_speed_percent", Float32, queue_size=1)
speed_pub = rospy.Publisher("rviz_speed", String, queue_size=1)
steer_pub = rospy.Publisher("rviz_steer", Float32, queue_size=1)

rospy.Subscriber('/commands/servo/position', Float64,SteerCallback,queue_size=1)
rospy.Subscriber('/sensors/core', VescStateStamped,SpeedCallback,queue_size=1)



r = rospy.Rate(30.0)
while not rospy.is_shutdown():
	r.sleep()
