#! /usr/bin/env python
# encoding: utf-8

"""
Date: 2022.11.29日创建
Detail: 把car_control11_26_final.py的中的部分函数分离出来
暂时未测试是否会影响运行
"""

import rospy
import time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose   # 可能待改
import math   # pi
import numpy as np


# 里程计的实时值
def odom_listener(print_message=False):
    odom_data = rospy.wait_for_message("/odom", Odometry, timeout=None)
    if print_message: 
        print("x: "+str(odom_data.pose.pose.position.x) +" y: "+str(odom_data.pose.pose.position.y))
    return odom_data

def pose_listener(print_message=False):
    pose_data = rospy.wait_for_message("/pose", Pose, timeout=None)
    if print_message: 
        print("x: "+str(pose_data.position.x) +"y: "+str(pose_data.position.y))
    return pose_data

# Imu的实时值
def Imu_listener(print_message=False):
    Imu_data = rospy.wait_for_message("/mobile_base/sensors/imu_data", Imu, timeout=None)
    if print_message: 
        # 旋转坐标系的值 rad
        print("Imu orient-x: %0.6f, orient-y: %0.6f, orient-z: %0.6f, orient-w: %0.6f\n", Imu_data.orientation.x, Imu_data.orientation.y, Imu_data.orientation.z, Imu_data.orientation.w)
        # 三个角速度值 
        # print("Imu agr_ve-x: %0.6f, agr_ve-y: %0.6f, agr_ve-z: %0.6f\n", Imu_data.angular_velocity.x, Imu_data.angular_velocity.y, Imu_data.angular_velocity.z)
        # 三个线速度值
        # print("Imu lin_ac-x: %0.6f, lin_ac-y: %0.6f, lin_ac-z: %0.6f\n", Imu_data.linear_acceleration.x, Imu_data.linear_acceleration.y, Imu_data.linear_acceleration.z)
    return Imu_data

# 雷达所检测出的实时值
def Distance_listener(print_message=False):
    Distance_data = rospy.wait_for_message("/points_distance", Imu, timeout=None)
    if print_message: 
        # 雷达的坐标值
        print("front-mean: %.4f, front-std: %.4f"%(Distance_data.orientation.x, Distance_data.orientation.y))
        print("left-mean: %.4f"%(Distance_data.orientation.z))
        print("right-mean: %.4f"%(Distance_data.angular_velocity.x))
        print("back-mean: %.4f, back-std: %.4f"%(Distance_data.linear_acceleration.x, Distance_data.linear_acceleration.y))

    return Distance_data.orientation.x, Distance_data.orientation.y, Distance_data.orientation.z, Distance_data.orientation.w, Distance_data.angular_velocity.x, Distance_data.angular_velocity.y, Distance_data.linear_acceleration.x, Distance_data.linear_acceleration.y


def odom_minus(start, end, print_message=True):
    pos_x = end.pose.pose.position.x - start.pose.pose.position.x
    pos_y = end.pose.pose.position.y - start.pose.pose.position.y
    if print_message:
        print("x distance: "+str(pos_x)) 
        print("y distance: "+str(pos_y)) 
    return [pos_x, pos_y]

def pose_minus(start, end, print_message=True):
    pos_x = end.position.x - start.position.x
    pos_y = end.position.y - start.position.y
    if print_message:
        print("x distance(pose): "+str(pos_x)) 
        print("y distance(pose): "+str(pos_y))
    return [pos_x, pos_y] 

def Imu_minus(start, end, print_message=True):
    angle = end.orientation.z - start.orientation.z
    if print_message:
        print("angle minus: "+str(angle)) 
    return angle

