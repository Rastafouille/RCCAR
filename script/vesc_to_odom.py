#!/usr/bin/env python3

import rospy
import tf
import time
import math


from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from vesc_msgs.msg import VescStateStamped
from std_msgs.msg import Float64

#Odometry
# std_msgs/Header header
#   uint32 seq
#   time stamp
#   string frame_id
# string child_frame_id
# geometry_msgs/PoseWithCovariance pose
#   geometry_msgs/Pose pose
#     geometry_msgs/Point position
#       float64 x
#       float64 y
#       float64 z
#     geometry_msgs/Quaternion orientation
#       float64 x
#       float64 y
#       float64 z
#       float64 w
#   float64[36] covariance
# geometry_msgs/TwistWithCovariance twist
#   geometry_msgs/Twist twist
#     geometry_msgs/Vector3 linear
#       float64 x
#       float64 y
#       float64 z
#     geometry_msgs/Vector3 angular
#       float64 x
#       float64 y
#       float64 z
#   float64[36] covariance

# VESC State
# std_msgs/Header header
#   uint32 seq
#   time stamp
#   string frame_id
# vesc_msgs/VescState state
#   int32 FAULT_CODE_NONE=0
#   int32 FAULT_CODE_OVER_VOLTAGE=1
#   int32 FAULT_CODE_UNDER_VOLTAGE=2
#   int32 FAULT_CODE_DRV8302=3
#   int32 FAULT_CODE_ABS_OVER_CURRENT=4
#   int32 FAULT_CODE_OVER_TEMP_FET=5
#   int32 FAULT_CODE_OVER_TEMP_MOTOR=6
#   float64 voltage_input
#   float64 temperature_pcb
#   float64 current_motor
#   float64 current_input
#   float64 speed
#   float64 duty_cycle
#   float64 charge_drawn
#   float64 charge_regen
#   float64 energy_drawn
#   float64 energy_regen
#   float64 displacement
#   float64 distance_traveled
#   int32 fault_code



rospy.init_node('vesc_to_odom_node2')
gear_ratio=86/27
wheel_diameter=0.088
angle_ratio = math.pi/4
nbre_pole_moteur=4
wheelbase=0.3
speed_to_erpm_gain=rospy.get_param("/speed_to_erpm_gain")
speed_to_erpm_offset=rospy.get_param("/speed_to_erpm_offset")
steering_angle_to_servo_gain=rospy.get_param("/steering_angle_to_servo_gain")
steering_angle_to_servo_offset=rospy.get_param("/steering_angle_to_servo_offset")

def erpm_to_speed(erpm):
	speed=erpm/nbre_pole_moteur*2*(2*pi*wheel_diameter/2)
	return speed 

def servo_to_angle(servo):
	angle=(servo-steering_angle_to_servo_offset)/steering_angle_to_servo_gain*angle_ratio
	return angle 

last_servo_position=0
x=0
y=0
theta=0
odom=Odometry()
odom.header.frame_id='odom'
odom.child_frame_id='base_link'

last_state=VescStateStamped()
global last_time
last_time=rospy.Time.now()

#listener = tf.TransformListener()


def StateCallback(data):
	speed=data.state.speed-speed_to_erpm_offset
	dt=data.header.stamp-last_time
	current_speed = erpm_to_speed(data.state.speed - speed_to_erpm_offset)
	current_angular_velocity = current_speed * math.tan(servo_to_angle(last_servo_position)) / wheelbase
	yaw= current_angular_velocity*dt
	x_dot = current_speed*math.cos(yaw)
	y_dot = current_speed *sin(yaw)
	x=x_dot*dt
	y=y_dot*dt
	odom.pose.pose.position.x += x
	odom.pose.pose.position.y += y
	odom.pose.pose.orientation.x = 0.0
	odom.pose.pose.orientation.y = 0.0
	odom.pose.pose.orientation.z = math.sin(yaw / 2.0)
	odom.pose.pose.orientation.w = math.cos(yaw / 2.0)
	#tolerance
	odom.pose.covariance[0]=0.2  #x
	odom.pose.covariance[7]=0.2  #y
	odom.pose.covariance[35]=0.4  #yaw
	#Vitesse
	odom.twist.twist.linear.x = current_speed
	odom.twist.twist.linear.y = 0.0
	odom.twist.twist.angular.z = current_angular_velocity

	odom.header.stamp=data.header.stamp
	odom_pub.publish(odom)
	last_time= data.header.stamp

def ServoCallback(data):
	last_servo_position=data.data


odom_pub = rospy.Publisher("odom", Odometry, queue_size=1)
rospy.Subscriber('/sensors/core', VescStateStamped,StateCallback,queue_size=1)
rospy.Subscriber('/sensors/servo_position_command', Float64,ServoCallback,queue_size=1)


r = rospy.Rate(30.0)
while not rospy.is_shutdown():
        r.sleep()

