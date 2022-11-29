#! /usr/bin/env python
# encoding: utf-8

"""
运行该文件, 必须运行以下launch文件:
roslaunch turn_on_dlrobot_robot navigation.launch
roslaunch lslidar_driver start.launch
roslaunch pointcloud_to_laserscan pointcloud_scan.launch
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
        print("Imu agr_ve-x: %0.6f, agr_ve-y: %0.6f, agr_ve-z: %0.6f\n", Imu_data.angular_velocity.x, Imu_data.angular_velocity.y, Imu_data.angular_velocity.z)
        # 三个线速度值
        print("Imu lin_ac-x: %0.6f, lin_ac-y: %0.6f, lin_ac-z: %0.6f\n", Imu_data.linear_acceleration.x, Imu_data.linear_acceleration.y, Imu_data.linear_acceleration.z)
    return Imu_data

# 雷达所检测出的实时值
def Distance_listener(print_message=False):
    Distance_data = rospy.wait_for_message("/points_distance", Imu, timeout=None)
    if print_message: 
        # 雷达的坐标值
        print("x-mean: %0.4f, x-std: %0.4f, y-mean: %0.4f, y-std: %0.4f\n", Distance_data.orientation.x, Distance_data.orientation.y, Distance_data.orientation.z, Distance_data.orientation.w)
    return Distance_data.orientation.x, Distance_data.orientation.y, Distance_data.orientation.z, Distance_data.orientation.w


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


"""测试可用"""    
def go_straight_x(cmd_vel_pub, speed, distance=None, odom_distance=None): 
    """
    x: 指在相对于坐标原点的x轴方向移动
    cmd_vel_pub: 发布话题的变量
    speed: 前进速度
    distance: 前进距离, 不使用里程计校正，通过代码计算
    odom_distance: 前进距离，使用里程计确定行走时间
    """
    odom_data_start = odom_listener(True) # 获取当前的里程计值
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
    if odom_distance is not None: # 使用里程计校正
        odom_data_end = odom_listener() # 获取当前的里程计值
        while (odom_data_end.pose.pose.position.x - odom_data_start.pose.pose.position.x) < odom_distance-0.1: # 注意这里的0.1可以修改
            cmd_vel_pub.publish(cmd_vel_msg)
            odom_data_end = odom_listener() # 获取当前的里程计值
        odom_minus(odom_data_start, odom_data_end)
    elif distance is not None: # 使用代码计算
        go_straight = (distance / speed) * send_rate - 1 # 
        # print(go_straight)
        while go_straight:
            cmd_vel_pub.publish(cmd_vel_msg)
            go_straight -= 1
            rate.sleep()
            odom_data_end = odom_listener() # 获取当前的里程计值
    else: # 默认走1m
        go_straight = (1 / speed) - 1 # 
        while go_straight:
            cmd_vel_pub.publish(cmd_vel_msg)  
            go_straight -= 1
            rate.sleep()
    odom_listener(True)
    return True

"""尚未验证，但应该可以用"""  
def go_straight_y(cmd_vel_pub, speed, distance=None, odom_distance=None): # 测试可用
    """
    y: 指在相对于开机点为坐标原点的y轴方向移动
    cmd_vel_pub: 发布话题的变量
    speed: 前进速度
    distance: 前进距离, 不使用里程计校正，通过代码计算
    odom_distance: 前进距离，使用里程计确定行走时间
    """
    odom_data_start = odom_listener(True) # 获取当前的里程计值, 并打印
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
    send_rate = 10
    rate = rospy.Rate(send_rate) # 0.1s
    if odom_distance is not None:
        odom_data_end = odom_listener() # 使用里程计校正前进距离
        while odom_minus(odom_data_start, odom_data_end, False)[0] < odom_distance: #尚未验证
            cmd_vel_pub.publish(cmd_vel_msg)
            rate.sleep()
            odom_data_end = odom_listener()
        odom_minus(odom_data_start, odom_data_end) # 打印里程计所计算的前进距离
    elif distance is not None: # 使用代码计算前进时间
        go_straight = (distance / speed) * send_rate - 1 # 
        while go_straight:
            cmd_vel_pub.publish(cmd_vel_msg)  
            go_straight -= 1
            rate.sleep()
    else: # 默认走1m
        go_straight = 9
        while go_straight:
            cmd_vel_pub.publish(cmd_vel_msg)
            rate.sleep()
            go_straight -= 1
    return True

"""还没写完"""  
def go_straight_angle(cmd_vel_pub, speed, distance=None): 
    odom_data_start = odom_listener() # 获取当前的里程计值
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
        odom_data_end = odom_listener() # 获取当前的里程计值
        while (odom_data_end.pose.pose.position.y - odom_data_start.pose.pose.position.y) < distance: ### 这里的条件还要修改
            cmd_vel_pub.publish(cmd_vel_msg)
            rate.sleep()
    else: # 默认走1m
        go_straight = 9
        while go_straight:
            cmd_vel_pub.publish(cmd_vel_msg)
            rate.sleep()
            go_straight -= 1
    return True

"""写完了还没有测试能不能用"""
def turn_left(cmd_vel_pub, speed, rotate_assert=None):
    assert (speed >= 0)  # 左转的转速必须是大于0
    Imu_data_start = Imu_listener(True) # 获取当前的Imu值
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
    send_rate = 10
    rate = rospy.Rate(send_rate) # 0.1s
    if rotate_assert is not None:
        Imu_data_end = odom_listener() # 获取当前的IMU值
        while (Imu_minus(Imu_data_start, Imu_data_end, False)) < rotate_assert:
            cmd_vel_pub.publish(cmd_vel_msg)
            rate.sleep()
            Imu_data_end = odom_listener() # 获取当前的IMU值
        Imu_minus(Imu_data_start, Imu_data_end, True)
    else: # 默认旋转90度
        go_straight = math.pi / (2 * speed) * send_rate + 2 # 后面的2自己修改, 用于校正摩擦力
        while go_straight:
            cmd_vel_pub.publish(cmd_vel_msg)
            rate.sleep()
            go_straight -= 1
    return True

"""写完了还没有测试能不能用"""
def turn_right(cmd_vel_pub, speed, rotate_assert=None):
    assert (speed <= 0)  # 右转的转速必须是小于0
    Imu_data_start = Imu_listener() # 获取当前的Imu值
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
    send_rate = 10
    rate = rospy.Rate(send_rate) # 0.1s
    if rotate_assert is not None:
        Imu_data_end = odom_listener() # 使用Imu确定旋转的角度和时间
        while Imu_minus(Imu_data_start, Imu_data_end, False) > -rotate_assert:
            cmd_vel_pub.publish(cmd_vel_msg)
            rate.sleep()
            Imu_data_end = odom_listener() # 获取当前的里程计值
        Imu_minus(Imu_data_start, Imu_data_end, True)
    else: # 默认旋转90度
        go_straight = math.pi/(2 * speed) * send_rate + 2 # 后面的2自己修改, 用于校正摩擦力
        while go_straight:
            cmd_vel_pub.publish(cmd_vel_msg)
            rate.sleep()
            go_straight -= 1
    return True

"""写完了还没有测试能不能用"""
def stop(cmd_vel_pub, time=2): 
    cmd_vel_msg = Twist()
    cmd_vel_msg.linear.x = 0
    cmd_vel_msg.linear.y = 0
    cmd_vel_msg.linear.z = 0

    cmd_vel_msg.angular.x = 0
    cmd_vel_msg.angular.y = 0
    cmd_vel_msg.angular.z = 0
    cmd_vel_pub.publish(cmd_vel_msg)
    start_time = time.time()
    rate = rospy.Rate(100)
    while time.time() - start_time < 2*1000:
        cmd_vel_pub.publish(cmd_vel_msg)
        rate.sleep()


def go_straight_x_until(cmd_vel_pub, distance, speed):
    """
    x: 指在相对于坐标原点的x轴方向移动
    cmd_vel_pub: 发布话题的变量
    speed: 前进速度
    distance: 前进距离, 不使用里程计校正，通过代码计算
    odom_distance: 前进距离，使用里程计确定行走时间
    """
    threshold = 2.0
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
    while True:
        cmd_vel_pub.publish(cmd_vel_msg)
        rate.sleep()
        x_means, x_std, y_mean, y_std = Distance_listener(False) # 获取当前的雷达测距值
        if y_means <= distance and y_std < threshold:
            break
    return True

def go_straight_y_until(cmd_vel_pub, distance, speed):
    """
    y: 指在相对于坐标原点的y轴方向移动
    cmd_vel_pub: 发布话题的变量
    speed: 前进速度
    distance: 前进距离, 不使用里程计校正，通过代码计算
    odom_distance: 前进距离，使用里程计确定行走时间
    """
    threshold = 2.0
    cmd_vel_msg = Twist()
    cmd_vel_msg.linear.x = 0
    cmd_vel_msg.linear.y = speed
    cmd_vel_msg.linear.z = 0

    cmd_vel_msg.angular.x = 0
    cmd_vel_msg.angular.y = 0
    cmd_vel_msg.angular.z = 0
    cmd_vel_pub.publish(cmd_vel_msg)
    rate = rospy.Rate(1)
    rate.sleep()
    send_rate = 10
    rate = rospy.Rate(send_rate) # 0.1s
    while True:
        cmd_vel_pub.publish(cmd_vel_msg)
        rate.sleep()
        x_means, x_std, y_mean, y_std = Distance_listener(False) # 获取当前的雷达测距值
        if x_means <= distance and x_std < threshold:
            break
    return True


def main(cmd_vel_pub):
    # 在相对于开机点为坐标原点的情况下，沿着地图的x轴方向前进2m
    go_straight_x(cmd_vel_pub, 0.5, distance=2) 
    # 直走直到前方距离是0.5m, 里面有个死循环，没有执行完的时候是不会跳出这个函数的
    go_straight_x_until(cmd_vel_pub, speed=0.5, distance=0.5) 
    turn_left(cmd_vel_pub, speed=0.5) # 左转90度
    # 在相对于开机点为坐标原点的情况下，沿着地图的y轴方向前进2m
    go_straight_y(cmd_vel_pub, speed=0.5, distance=2) 


if __name__ == '__main__':
    try:
        # ROS节点初始化
        rospy.init_node('cmd_vel_test', anonymous=True)
        cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=100)
        main(cmd_vel_pub)
    except rospy.ROSInterruptException:
        pass
