#! /usr/bin/env python
# encoding: utf-8

from tkinter import N
from tkinter.messagebox import NO
from torch import _register_default_hooks
import rospy
import time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import math   # pi

def odom_listener():
    odom_data = rospy.wait_for_message("/odom", Odometry, timeout=None)
    print("x: "+str(odom_data.pose.pose.position.x) +"y: "+str(odom_data.pose.pose.position.y))
    return odom_data

def pose_listener():
    pose_data = rospy.wait_for_message("/pose", Pose, timeout=None)
    print("x: "+str(pose_data.position.x) +"y: "+str(pose_data.position.y))
    return pose_data

def Imu_listener():
    Imu_data = rospy.wait_for_message("/mobile_base/sensors/imu_data", Imu, timeout=None)
    # ��ת����ϵ��ֵ rad
    print("Imu orient-x: %0.6f, orient-y: %0.6f, orient-z: %0.6f, orient-w: %0.6f\n", Imu_data.orientation.x, Imu_data.orientation.y, Imu_data.orientation.z, Imu_data.orientation.w)
    # �������ٶ�ֵ 
    print("Imu agr_ve-x: %0.6f, agr_ve-y: %0.6f, agr_ve-z: %0.6f\n", Imu_data.angular_velocity.x, Imu_data.angular_velocity.y, Imu_data.angular_velocity.z)
    # �������ٶ�ֵ
    print("Imu lin_ac-x: %0.6f, lin_ac-y: %0.6f, lin_ac-z: %0.6f\n", Imu_data.linear_acceleration.x, Imu_data.linear_acceleration.y, Imu_data.linear_acceleration.z)
    return Imu_data

def odom_minus(start, end):
    pos_x = end.pose.pose.position.x - start.pose.pose.position.x
    pos_y = end.pose.pose.position.y - start.pose.pose.position.y
    print("x distance: "+str(pos_x)) 
    print("y distance: "+str(pos_y)) 

def pose_minus(start, end):
    pos_x = end.position.x - start.position.x
    pos_y = end.position.y - start.position.y
    print("x distance(pose): "+str(pos_x)) 
    print("y distance(pose): "+str(pos_y)) 

    
def go_straight_x(cmd_vel_pub, speed, distance=None, odom_distance=None):
    odom_data_start = odom_listener() # ��ȡ��ǰ����̼�ֵ
    cmd_vel_msg = Twist()
    cmd_vel_msg.linear.x = speed
    cmd_vel_msg.linear.y = 0
    cmd_vel_msg.linear.z = 0

    cmd_vel_msg.angular.x = 0
    cmd_vel_msg.angular.y = 0
    cmd_vel_msg.angular.z = 0
    cmd_vel_pub.publish(cmd_vel_msg)
    rate = rospy.Rate(1)
    rate.sleep()
    send_rate = 10
    rate = rospy.Rate(send_rate) # 0.1s
    if odom_distance is not None:
        odom_data_end = odom_listener() # ��ȡ��ǰ����̼�ֵ
        while (odom_data_end.pose.pose.position.x - odom_data_start.pose.pose.position.x) < odom_distance-0.1: # ע�������0.1�����޸�
            cmd_vel_pub.publish(cmd_vel_msg)
            rate.sleep()
            odom_data_end = odom_listener() # ��ȡ��ǰ����̼�ֵ
    elif distance is not None: # Ĭ����1m
        go_straight = (distance / speed) * send_rate - 1 # 
        print(go_straight)
        while go_straight:
            cmd_vel_pub.publish(cmd_vel_msg)
            go_straight -= 1
            rate.sleep()
            odom_data_end = odom_listener() # ��ȡ��ǰ����̼�ֵ
    else: # Ĭ����1m
        go_straight = (1 / speed) - 1 # 
        while go_straight:
            cmd_vel_pub.publish(cmd_vel_msg)  
            go_straight -= 1
            rate.sleep()
    return True

def go_straight_y(cmd_vel_pub, speed, distance=None, odom_distance=None):
    odom_data_start = odom_listener() # ��ȡ��ǰ����̼�ֵ
    cmd_vel_msg = Twist()
    cmd_vel_msg.linear.x = 0
    cmd_vel_msg.linear.y = 0
    cmd_vel_msg.linear.z = 0

    cmd_vel_msg.angular.x = 0
    cmd_vel_msg.angular.y = 0
    cmd_vel_msg.angular.z = 0
    cmd_vel_pub.publish(cmd_vel_msg)
    rate = rospy.Rate(1)
    rate.sleep()
    rate = rospy.Rate(10) # 0.1s
    if odom_distance is not None:
        odom_data_end = odom_listener() # ��ȡ��ǰ����̼�ֵ
        while (odom_data_end.pose.pose.position.y - odom_data_start.pose.pose.position.y) < odom_distance:
            cmd_vel_pub.publish(cmd_vel_msg)
            rate.sleep()
    elif distance is not None: # Ĭ����1m
        go_straight = 9
        while go_straight:
            cmd_vel_pub.publish(cmd_vel_msg)
            rate.sleep()
            go_straight -= 1
    return True

def go_straight_angle(cmd_vel_pub, speed, distance=None):
    odom_data_start = odom_listener() # ��ȡ��ǰ����̼�ֵ
    cmd_vel_msg = Twist()
    cmd_vel_msg.linear.x = 0
    cmd_vel_msg.linear.y = 0
    cmd_vel_msg.linear.z = 0

    cmd_vel_msg.angular.x = 0
    cmd_vel_msg.angular.y = 0
    cmd_vel_msg.angular.z = 0
    cmd_vel_pub.publish(cmd_vel_msg)
    rate = rospy.Rate(1)
    rate.sleep()
    rate = rospy.Rate(10) # 0.1s
    if distance is not None:
        odom_data_end = odom_listener() # ��ȡ��ǰ����̼�ֵ
        while (odom_data_end.pose.pose.position.y - odom_data_start.pose.pose.position.y) < distance: ### �����������Ҫ�޸�
            cmd_vel_pub.publish(cmd_vel_msg)
            rate.sleep()
    else: # Ĭ����1m
        go_straight = 9
        while go_straight:
            cmd_vel_pub.publish(cmd_vel_msg)
            rate.sleep()
            go_straight -= 1
    return True

def turn_left(cmd_vel_pub, speed, rotate_assert=None):
    assert (speed >= 0)  # ��ת��ת�ٱ����Ǵ���0
    Imu_data_start = Imu_listener() # ��ȡ��ǰ��Imuֵ
    cmd_vel_msg = Twist()
    cmd_vel_msg.linear.x = 0
    cmd_vel_msg.linear.y = 0
    cmd_vel_msg.linear.z = 0

    cmd_vel_msg.angular.x = 0
    cmd_vel_msg.angular.y = 0
    cmd_vel_msg.angular.z = speed
    cmd_vel_pub.publish(cmd_vel_msg)
    rate = rospy.Rate(1)
    rate.sleep()
    rate = rospy.Rate(10) # 0.1s
    if rotate_assert is not None:
        Imu_data_end = odom_listener() # ��ȡ��ǰ����̼�ֵ
        while (Imu_data_end.orientation.z - Imu_data_start.orientation.z) < rotate_assert:
            cmd_vel_pub.publish(cmd_vel_msg)
            rate.sleep()
    else: # Ĭ����ת90��
        go_straight = math.pi / (2*speed) * rate
        while go_straight:
            cmd_vel_pub.publish(cmd_vel_msg)
            rate.sleep()
            go_straight -= 1
    return True

def turn_right(cmd_vel_pub, rotate_assert=None):
    assert (speed <= 0)  # ��ת��ת�ٱ����Ǵ���0
    Imu_data_start = Imu_listener() # ��ȡ��ǰ��Imuֵ
    cmd_vel_msg = Twist()
    cmd_vel_msg.linear.x = 0
    cmd_vel_msg.linear.y = 0
    cmd_vel_msg.linear.z = 0

    cmd_vel_msg.angular.x = 0
    cmd_vel_msg.angular.y = 0
    cmd_vel_msg.angular.z = speed
    cmd_vel_pub.publish(cmd_vel_msg)
    rate = rospy.Rate(1)
    rate.sleep()
    rate = rospy.Rate(10) # 0.1s
    if rotate_assert is not None:
        Imu_data_end = odom_listener() # ��ȡ��ǰ����̼�ֵ
        while (Imu_data_end.orientation.z - Imu_data_start.orientation.z) < rotate_assert:
            cmd_vel_pub.publish(cmd_vel_msg)
            rate.sleep()
    else: # Ĭ����ת90��
        go_straight = math.pi/speed*rate + 2 # �����2�Լ��޸�
        while go_straight:
            cmd_vel_pub.publish(cmd_vel_msg)
            rate.sleep()
            go_straight -= 1
    return True

#def stop(cmd_vel_pub, speed, distance=None):


def cmd_vel_publisher():
	# ROS�ڵ��ʼ��
    rospy.init_node('cmd_vel_test', anonymous=True)

    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=100)
    go_straight_x(cmd_vel_pub, 0.5, distance=2)

	

if __name__ == '__main__':
    try:
        cmd_vel_publisher()
    except rospy.ROSInterruptException:
        pass
