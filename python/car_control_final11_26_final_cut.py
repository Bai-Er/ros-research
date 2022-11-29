#! /usr/bin/env python
# encoding: utf-8

"""
Last updated time: 2022.11.26下午
Detail: 2022.11.26去鸽子场，最后保存的最新代码
运行该文件, 必须运行以下launch文件:
    roslaunch ge_detect_and_run.launch
注意：此文件配合car_function.py使用
"""

import rospy
import time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose   # 可能待改
import math   # pi
import numpy as np

def Center_detect_11(cmd_vel_pub, road_width=1.1):
    """
    校正车辆行驶的方向
    """
    threshold = 0.4
    _, _, left_distance, _, right_distance, _, _, _ = Distance_listener(True) # 获取当前的雷达测距值
    left_distance = -1.0 * left_distance
    # assert (abs(left_distance + right_distance - road_width) < threshold)
    cmd_vel_msg = Twist()
    cmd_vel_msg.linear.x = 0
    cmd_vel_msg.linear.y = 0
    cmd_vel_msg.linear.z = 0

    cmd_vel_msg.angular.x = 0
    cmd_vel_msg.angular.y = 0
    send_rate = 10
    rate = rospy.Rate(send_rate) # 0.1s
    # 如果左转，则转速z>0
    if math.isnan(right_distance):
       cmd_vel_msg.angular.z = - 0.32
    elif math.isnan(left_distance):
        cmd_vel_msg.angular.z = 0.32  

    elif abs(left_distance - right_distance) < road_width / 20: # 在一定时长内不做矫正
        print("policy-20")
        cmd_vel_msg.angular.z = 0
        
    elif abs(left_distance - right_distance) < road_width / 10: # 稍微有一点偏航
        print("policy-10")
        kp = 0.49
        cmd_vel_msg.angular.z = (left_distance - right_distance) * kp # 调节比例系数
    elif abs(left_distance - right_distance) < road_width / 7:
        print("policy-7")
        kp = 0.52
        cmd_vel_msg.angular.z = (left_distance - right_distance) * kp
    elif abs(left_distance - right_distance) < road_width / 5:
        print("policy-5")
        kp = 0.55
        cmd_vel_msg.angular.z = (left_distance - right_distance) * kp
    elif abs(left_distance - right_distance) < road_width / 3:
        print("policy-3")
        kp = 0.59
        cmd_vel_msg.angular.z = (left_distance - right_distance) * kp
    elif abs(left_distance - right_distance) < road_width / 2:
        print("policy-2")
        kp = 0.63
        cmd_vel_msg.angular.z = (left_distance - right_distance) * kp

   # elif abs(left_distance - right_distance) < road_width / 5:  # 
   #     kp = 1
   #    cmd_vel_msg.angular.z = (left_distance - right_distance) * kp # 调节比例系数

   # elif abs(left_distance - right_distance) < road_width / 2:
   #     kp = 1
   #     cmd_vel_msg.angular.z = (left_distance - right_distance) * kp # 调节比例系数

    go_rotate = 10
    while go_rotate:
        cmd_vel_pub.publish(cmd_vel_msg)  
        go_rotate -= 1
	# print(go_rotate)
        rate.sleep()
    
    return False


def Center_detect_12(cmd_vel_pub, road_width=1.2):
    """
    校正车辆行驶的方向用
    """
    threshold = 0.4
    _, _, left_distance, _, right_distance, _, _, _ = Distance_listener(True) # 获取当前的雷达测距值
    left_distance = -1.0 * left_distance
    
    # assert (abs(left_distance + right_distance - road_width) < threshold)

    cmd_vel_msg = Twist()
    cmd_vel_msg.linear.x = 0
    cmd_vel_msg.linear.y = 0
    cmd_vel_msg.linear.z = 0

    cmd_vel_msg.angular.x = 0
    cmd_vel_msg.angular.y = 0
    send_rate = 10
    rate = rospy.Rate(send_rate) # 0.1s
    # 如果左转，则转速z>0
    if math.isnan(right_distance):
       cmd_vel_msg.angular.z = - 0.32
    elif math.isnan(left_distance):
        cmd_vel_msg.angular.z = 0.32  

    elif abs(left_distance - right_distance) < road_width / 18: # 在一定时长内不做矫正
        print("policy-20")
        cmd_vel_msg.angular.z = 0
        
    elif abs(left_distance - right_distance) < road_width / 10: # 稍微有一点偏航
        print("policy-10")
        kp = 0.47
        cmd_vel_msg.angular.z = (left_distance - right_distance) * kp # 调节比例系数
    elif abs(left_distance - right_distance) < road_width / 7:
        print("policy-7")
        kp = 0.47
        cmd_vel_msg.angular.z = (left_distance - right_distance) * kp
    elif abs(left_distance - right_distance) < road_width / 5:
        print("policy-5")
        kp = 0.50
        cmd_vel_msg.angular.z = (left_distance - right_distance) * kp
    elif abs(left_distance - right_distance) < road_width / 3:
        print("policy-3")
        kp = 0.54 
        cmd_vel_msg.angular.z = (left_distance - right_distance) * kp
    elif abs(left_distance - right_distance) < road_width / 2:
        print("policy-2")
        kp = 0.57
        cmd_vel_msg.angular.z = (left_distance - right_distance) * kp

   # elif abs(left_distance - right_distance) < road_width / 5:  # 
   #     kp = 1
   #    cmd_vel_msg.angular.z = (left_distance - right_distance) * kp # 调节比例系数

   # elif abs(left_distance - right_distance) < road_width / 2:
   #     kp = 1
   #     cmd_vel_msg.angular.z = (left_distance - right_distance) * kp # 调节比例系数

    go_rotate = 10
    while go_rotate:
        cmd_vel_pub.publish(cmd_vel_msg)  
        go_rotate -= 1
	# print(go_rotate)
        rate.sleep()
    
    return False



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
        go_straight = int ((distance / speed) * send_rate - 1) # 
        
        while go_straight:
            cmd_vel_pub.publish(cmd_vel_msg)
            go_straight -= 1
            # print(go_straight)
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

"""写完了"""  
def go_straight_angle(cmd_vel_pub, speed, distance=None): 
    odom_data_start = odom_listener() # 获取当前的里程计值
    x_pose = odom_data_start.pose.pose.position.x
    y_pose = odom_data_start.pose.pose.position.y
    cmd_vel_msg = Twist()
    cmd_vel_msg.linear.x = speed * x_pose / math.sqrt(x_pose **2 + y_pose ** 2)
    cmd_vel_msg.linear.y = speed * y_pose / math.sqrt(x_pose **2 + y_pose ** 2)
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

"""测试可用"""
def turn_left(cmd_vel_pub, speed, add=10, rotate_assert=None):
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
        print("test")
        go_straight = int (math.pi / (2 * speed) * send_rate + add) # 后面的2自己修改, 用于校正摩擦力
        print(go_straight)
        while go_straight:
            cmd_vel_pub.publish(cmd_vel_msg)
            rate.sleep()
            go_straight -= 1
        print(go_straight)
    return True

"""写完了还没有测试能不能用"""
def turn_right(cmd_vel_pub, speed, add=13, rotate_assert=None):
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
        go_straight = int(-1 * math.pi/(2 * speed) * send_rate + int(add)) # 后面的2自己修改, 用于校正摩擦力
        while go_straight:
            cmd_vel_pub.publish(cmd_vel_msg)
            rate.sleep()
            go_straight -= 1
            print(go_straight)
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
    end_time = time.time()
    while end_time - start_time < 2*1000:
        cmd_vel_pub.publish(cmd_vel_msg)
        rate.sleep()
    end_time = time.time()


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
        front_mean, front_std, _, _, _, _, _, _ = Distance_listener(False) # 获取当前的雷达测距值
        if front_mean <= distance and front_std < threshold:
            print("go_straight_x_until has done {}".format(distance))
            break
    return True


def main(cmd_vel_pub):

    # five road
    front_go = 30  # need to test
    while front_go:
        go_straight_x(cmd_vel_pub, 0.7, distance=2) 
    	# 直走直到前方距离是0.5m, 里面有个死循环，没有执行完的时候是不会跳出这个函数的
        Center_detect_11(cmd_vel_pub)
        front_go -= 1
        print(front_go)

    go_straight_x(cmd_vel_pub, 0.7, distance=1.2)
    go_straight_x(cmd_vel_pub, 0.7, distance=2) 
    go_straight_x(cmd_vel_pub, 0.7, distance=2) 
                                      
    go_straight_x_until(cmd_vel_pub, speed=0.1, distance=1.4) 
    turn_left(cmd_vel_pub, speed=0.5) # 左转90度
    go_straight_x(cmd_vel_pub, 0.7, distance=2.7)
    turn_left(cmd_vel_pub, speed=0.5) # 左转90度
    go_straight_x(cmd_vel_pub, 0.7, distance=3)
    stop(cmd_vel_pub, time=10)
    
    # four
    front_go = 13
    while front_go:
        go_straight_x(cmd_vel_pub, 0.7, distance=1.5) 
	    # 直走直到前方距离是0.5m, 里面有个死循环，没有执行完的时候是不会跳出这个函数的
        Center_detect_12(cmd_vel_pub, road_width = 1.2)
        front_go -= 1
        print(front_go)

    
    go_straight_x(cmd_vel_pub, 0.7, distance=1.2)
    #------
    go_straight_x_until(cmd_vel_pub, speed=0.1, distance=0.7)  # 0.7
    turn_right(cmd_vel_pub, speed= - 0.5, add=15) # 左转90度
    go_straight_x(cmd_vel_pub, 0.7, distance=2.6)
    turn_right(cmd_vel_pub, speed= - 0.5, add=13) # 左转90度
    go_straight_x(cmd_vel_pub, 0.7, distance=2) 
    stop(cmd_vel_pub, time=10)
    print("----end circle----")

    # three
    front_go = 106
    while front_go:
        go_straight_x(cmd_vel_pub, 0.7, distance=0.5) 
	   # 直走直到前方距离是0.5m, 里面有个死循环，没有执行完的时候是不会跳出这个函数的
        Center_detect_11(cmd_vel_pub)
        front_go -= 1
        print(front_go)
    print("----end circle----")

    go_straight_x_until(cmd_vel_pub, speed=0.1, distance=1.5)
    turn_left(cmd_vel_pub, speed=0.5, add=8) # 左转90度
    go_straight_x(cmd_vel_pub, 0.7, distance=2.6)
    turn_left(cmd_vel_pub, speed=0.5, add=9) # 左转90度
    go_straight_x(cmd_vel_pub, 0.7, distance=1.5)

    # two
    front_go = 120
    while front_go:
        go_straight_x(cmd_vel_pub, 0.7, distance=0.4) 
	    # 直走直到前方距离是0.5m, 里面有个死循环，没有执行完的时候是不会跳出这个函数的
        Center_detect_12(cmd_vel_pub, road_width = 1.2)
        front_go -= 1
        print(front_go)

    go_straight_x_until(cmd_vel_pub, speed=0.1, distance=0.7)
    turn_right(cmd_vel_pub, speed=-0.5, add=15) # 左转90度
    go_straight_x(cmd_vel_pub, 0.7, distance=2.6)
    turn_right(cmd_vel_pub, speed=-0.5, add=13) # 左转90度
    go_straight_x(cmd_vel_pub, 0.7, distance=2)
    print("----end circle----"+front_go)

	
    # one
    front_go = 58
    while True:
        go_straight_x(cmd_vel_pub, 0.7, distance=1) 
	    # 直走直到前方距离是0.5m, 里面有个死循环，没有执行完的时候是不会跳出这个函数的
        Center_detect_11(cmd_vel_pub)
        front_go -= 1
        print(front_go)
    print("----end circle----")
    print("end")
    
    

if __name__ == '__main__':
    try:
        # ROS节点初始化
        rospy.init_node('cmd_vel_test', anonymous=True)
        cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=100)
        main(cmd_vel_pub)
    except rospy.ROSInterruptException:
        pass
