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

class vesc_to_odom:
	def __init__(self):
		rospy.init_node('vesc_to_odom_node2')
		self.gear_ratio=86/27
		self.wheel_diameter=0.088
		self.angle_ratio =30*2*math.pi/360
		self.nbre_pole_moteur=4
		self.wheelbase=0.3
		self.speed_to_erpm_gain=rospy.get_param("/speed_to_erpm_gain")
		self.speed_to_erpm_offset=rospy.get_param("/speed_to_erpm_offset")
		self.steering_angle_to_servo_gain=rospy.get_param("/steering_angle_to_servo_gain")
		self.steering_angle_to_servo_offset=rospy.get_param("/steering_angle_to_servo_offset")

		self.last_servo_position=0
		self.x=0
		self.y=0
		self.theta=0
		self.odom=Odometry()
		self.odom.header.frame_id='odom'
		self.odom.child_frame_id='base_link'

		self.last_state=VescStateStamped()
		self.last_time=rospy.Time.now()

		self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=1)
		rospy.Subscriber('/sensors/core', VescStateStamped,self.StateCallback,queue_size=1)
		rospy.Subscriber('/sensors/servo_position_command', Float64,self.ServoCallback,queue_size=1)



	def erpm_to_speed(self,erpm):
		speed=erpm/self.nbre_pole_moteur*2*(2*math.pi*self.wheel_diameter/2)/self.gear_ratio/60
		return speed 

	def servo_to_angle(self,servo):
		angle=(servo-self.steering_angle_to_servo_offset)/self.steering_angle_to_servo_gain*self.angle_ratio
		print('angle=',angle*180/math.pi)
		return angle 


	def StateCallback(self,data):
		
		dt=data.header.stamp-self.last_time
		#print ('dt=', dt.to_sec())
		current_speed = self.erpm_to_speed(-data.state.speed)# - self.speed_to_erpm_offset)
		print('current_speed=',current_speed)
		current_angular_velocity = current_speed * math.tan(self.servo_to_angle(self.last_servo_position)) / self.wheelbase
		print('current_angular_velocity=',current_angular_velocity)
		self.theta+= current_angular_velocity*dt.to_sec()
		print('theta=',self.theta)
		x_dot = current_speed*math.cos(self.theta)
		y_dot = current_speed *math.sin(self.theta)
		x=x_dot*dt.to_sec()
		y=y_dot*dt.to_sec()
		self.odom.pose.pose.position.x += x
		self.odom.pose.pose.position.y += y
		self.odom.pose.pose.orientation.x = 0.0
		self.odom.pose.pose.orientation.y = 0.0
		self.odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
		self.odom.pose.pose.orientation.w = math.cos(self.theta/ 2.0)
			#tolerance
		self.odom.pose.covariance[0]=0.2  #x
		self.odom.pose.covariance[7]=0.2  #y
		self.odom.pose.covariance[35]=0.4  #theta
			#Vitesse
		self.odom.twist.twist.linear.x = current_speed
		self.odom.twist.twist.linear.y = 0.0
		self.odom.twist.twist.angular.z = current_angular_velocity

		self.odom.header.stamp=data.header.stamp
		self.odom_pub.publish(self.odom)
		self.last_time= data.header.stamp

	def ServoCallback(self,data):
		self.last_servo_position=data.data

		


	


if __name__ == '__main__':
    try:
        vesc_to_odom=vesc_to_odom() #our main function
    except rospy.ROSInterruptException:
        print ('interrupted !')
    rate = rospy.Rate(30) #hz

    while not rospy.is_shutdown():
        rate.sleep()

