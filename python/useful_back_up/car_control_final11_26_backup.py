#! /usr/bin/env python
# encoding: utf-8

"""
���и��ļ�, ������������launch�ļ�:
roslaunch turn_on_dlrobot_robot navigation.launch
roslaunch lslidar_driver start.launch
roslaunch pointcloud_to_laserscan pointcloud_scan.launch
# �����������launch�ļ�
roslaunch turn_on_dlrobot_robot ge(Tab��)
"""

import rospy
import time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose   # ���ܴ���

import math   # pi
import numpy as np

# ��̼Ƶ�ʵʱֵ
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

# Imu��ʵʱֵ
def Imu_listener(print_message=False):
    Imu_data = rospy.wait_for_message("/mobile_base/sensors/imu_data", Imu, timeout=None)
    if print_message: 
        # ��ת����ϵ��ֵ rad
        print("Imu orient-x: %0.6f, orient-y: %0.6f, orient-z: %0.6f, orient-w: %0.6f\n", Imu_data.orientation.x, Imu_data.orientation.y, Imu_data.orientation.z, Imu_data.orientation.w)
        # �������ٶ�ֵ 
        # print("Imu agr_ve-x: %0.6f, agr_ve-y: %0.6f, agr_ve-z: %0.6f\n", Imu_data.angular_velocity.x, Imu_data.angular_velocity.y, Imu_data.angular_velocity.z)
        # �������ٶ�ֵ
        # print("Imu lin_ac-x: %0.6f, lin_ac-y: %0.6f, lin_ac-z: %0.6f\n", Imu_data.linear_acceleration.x, Imu_data.linear_acceleration.y, Imu_data.linear_acceleration.z)
    return Imu_data

# �״���������ʵʱֵ
def Distance_listener(print_message=False):
    Distance_data = rospy.wait_for_message("/points_distance", Imu, timeout=None)
    if print_message: 
        # �״������ֵ
        print("front-mean: %0.4f, front-std: %0.4f", Distance_data.orientation.x, Distance_data.orientation.y)
        print("left-mean: %0.4f, left-std: %0.4f", Distance_data.orientation.z, Distance_data.orientation.w)
        print("right-mean: %0.4f, right-std: %0.4f", Distance_data.angular_velocity.x, Distance_data.angular_velocity.y)
        print("back-mean: %0.4f, back-std: %0.4f", Distance_data.linear_acceleration.x, Distance_data.linear_acceleration.y)

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

def Center_detect(cmd_vel_pub, road_width, ):
    """
    У��������ʻ�ķ�����

    """
    threshold = 0.5
    _, _, left_distance, left_std, right_distance, right_std, _, _ = Distance_listener(False) # ��ȡ��ǰ���״���ֵ
    
    assert (abs(left_distance + right_distance - road_width) < threshold) , "function: Center_detect can not be used."
    cmd_vel_msg = Twist()
    cmd_vel_msg.linear.x = 0
    cmd_vel_msg.linear.y = 0
    cmd_vel_msg.linear.z = 0

    cmd_vel_msg.angular.x = 0
    cmd_vel_msg.angular.y = 0
    send_rate = 10
    rate = rospy.Rate(send_rate) # 0.1s
    # �����ת����ת��z>0

    if abs(left_distance - right_distance) < road_width / 20: # ��һ��ʱ���ڲ�������
        cmd_vel_msg.angular.z = 0
        
    elif abs(left_distance - right_distance) < road_width / 10: # ��΢��һ��ƫ��
        kp = 0.5
        cmd_vel_msg.angular.z = (left_distance - right_distance) * kp # ���ڱ���ϵ��

    elif abs(left_distance - right_distance) < road_width / 5:  # 
        kp = 1
        cmd_vel_msg.angular.z = (left_distance - right_distance) * kp # ���ڱ���ϵ��

    elif abs(left_distance - right_distance) < road_width / 2:
        kp = 1
        cmd_vel_msg.angular.z = (left_distance - right_distance) * kp # ���ڱ���ϵ��

    go_rotate = 10
    while go_rotate:
        cmd_vel_pub.publish(cmd_vel_msg)  
        go_rotate -= 1
        rate.sleep()

    _, _, left_distance, left_std, right_distance, right_std, _, _ = Distance_listener(False) # ��ȡ��ǰ���״���ֵ

    if abs(left_distance - right_distance) < road_width / 20:

        print("correct finish!")
    
    return False


"""���Կ���"""    
def go_straight_x(cmd_vel_pub, speed, distance=None, odom_distance=None): 
    """
    x: ָ�����������ԭ���x�᷽���ƶ�
    cmd_vel_pub: ��������ı���
    speed: ǰ���ٶ�
    distance: ǰ������, ��ʹ����̼�У����ͨ���������
    odom_distance: ǰ�����룬ʹ����̼�ȷ������ʱ��
    """
    odom_data_start = odom_listener(True) # ��ȡ��ǰ����̼�ֵ
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
    if odom_distance is not None: # ʹ����̼�У��
        odom_data_end = odom_listener() # ��ȡ��ǰ����̼�ֵ
        while (odom_data_end.pose.pose.position.x - odom_data_start.pose.pose.position.x) < odom_distance-0.1: # ע�������0.1�����޸�
            cmd_vel_pub.publish(cmd_vel_msg)
            odom_data_end = odom_listener() # ��ȡ��ǰ����̼�ֵ
        odom_minus(odom_data_start, odom_data_end)
    elif distance is not None: # ʹ�ô������
        go_straight = (distance / speed) * send_rate - 1 # 
        # print(go_straight)
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
    odom_listener(True)
    return True

"""��δ��֤����Ӧ�ÿ�����"""  
def go_straight_y(cmd_vel_pub, speed, distance=None, odom_distance=None): # ���Կ���
    """
    y: ָ������ڿ�����Ϊ����ԭ���y�᷽���ƶ�
    cmd_vel_pub: ��������ı���
    speed: ǰ���ٶ�
    distance: ǰ������, ��ʹ����̼�У����ͨ���������
    odom_distance: ǰ�����룬ʹ����̼�ȷ������ʱ��
    """
    odom_data_start = odom_listener(True) # ��ȡ��ǰ����̼�ֵ, ����ӡ
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
        odom_data_end = odom_listener() # ʹ����̼�У��ǰ������
        while odom_minus(odom_data_start, odom_data_end, False)[0] < odom_distance: #��δ��֤
            cmd_vel_pub.publish(cmd_vel_msg)
            rate.sleep()
            odom_data_end = odom_listener()
        odom_minus(odom_data_start, odom_data_end) # ��ӡ��̼��������ǰ������
    elif distance is not None: # ʹ�ô������ǰ��ʱ��
        go_straight = (distance / speed) * send_rate - 1 # 
        while go_straight:
            cmd_vel_pub.publish(cmd_vel_msg)  
            go_straight -= 1
            rate.sleep()
    else: # Ĭ����1m
        go_straight = 9
        while go_straight:
            cmd_vel_pub.publish(cmd_vel_msg)
            rate.sleep()
            go_straight -= 1
    return True

"""д����"""  
def go_straight_angle(cmd_vel_pub, speed, distance=None): 
    odom_data_start = odom_listener() # ��ȡ��ǰ����̼�ֵ
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

"""���Կ���"""
def turn_left(cmd_vel_pub, speed, rotate_assert=None):
    assert (speed >= 0)  # ��ת��ת�ٱ����Ǵ���0
    Imu_data_start = Imu_listener(True) # ��ȡ��ǰ��Imuֵ
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
        Imu_data_end = odom_listener() # ��ȡ��ǰ��IMUֵ
        while (Imu_minus(Imu_data_start, Imu_data_end, False)) < rotate_assert:
            cmd_vel_pub.publish(cmd_vel_msg)
            rate.sleep()
            Imu_data_end = odom_listener() # ��ȡ��ǰ��IMUֵ
        Imu_minus(Imu_data_start, Imu_data_end, True)
    else: # Ĭ����ת90��
        print("test")
        go_straight = int (math.pi / (2 * speed) * send_rate + 2) # �����2�Լ��޸�, ����У��Ħ����
        print(go_straight)
        while go_straight:
            cmd_vel_pub.publish(cmd_vel_msg)
            rate.sleep()
            go_straight -= 1
        print(go_straight)
    return True

"""д���˻�û�в����ܲ�����"""
def turn_right(cmd_vel_pub, speed, rotate_assert=None):
    assert (speed <= 0)  # ��ת��ת�ٱ�����С��0
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
    send_rate = 10
    rate = rospy.Rate(send_rate) # 0.1s
    if rotate_assert is not None:
        Imu_data_end = odom_listener() # ʹ��Imuȷ����ת�ĽǶȺ�ʱ��
        while Imu_minus(Imu_data_start, Imu_data_end, False) > -rotate_assert:
            cmd_vel_pub.publish(cmd_vel_msg)
            rate.sleep()
            Imu_data_end = odom_listener() # ��ȡ��ǰ����̼�ֵ
        Imu_minus(Imu_data_start, Imu_data_end, True)
    else: # Ĭ����ת90��
        go_straight = int(math.pi/(2 * speed) * send_rate + 2) # �����2�Լ��޸�, ����У��Ħ����
        while go_straight:
            cmd_vel_pub.publish(cmd_vel_msg)
            rate.sleep()
            go_straight -= 1
    return True

"""д���˻�û�в����ܲ�����"""
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
    x: ָ�����������ԭ���x�᷽���ƶ�
    cmd_vel_pub: ��������ı���
    speed: ǰ���ٶ�
    distance: ǰ������, ��ʹ����̼�У����ͨ���������
    odom_distance: ǰ�����룬ʹ����̼�ȷ������ʱ��
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
        front_mean, front_std, _, _, _, _, _, _ = Distance_listener(False) # ��ȡ��ǰ���״���ֵ
        if front_mean <= distance and front_std < threshold:
            print("go_straight_x_until has done {}".format(distance))
            break
    return True

def go_straight_y_until(cmd_vel_pub, distance, speed):
    """
    x: ָ�����������ԭ���y�᷽���ƶ�
    cmd_vel_pub: ��������ı���
    speed: ǰ���ٶ�
    distance: ǰ������, ��ʹ����̼�У����ͨ���������
    odom_distance: ǰ�����룬ʹ����̼�ȷ������ʱ��
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
        front_mean, front_std, _, _, _, _, _, _ = Distance_listener(False) # ��ȡ��ǰ���״���ֵ
        if front_mean <= distance and front_std < threshold:
            print("go_straight_y_until has done {}".format(distance))
            break
    return True

def main(cmd_vel_pub):
    # ������ڿ�����Ϊ����ԭ�������£����ŵ�ͼ��x�᷽��ǰ��2m
    #go_straight_x(cmd_vel_pub, 0.4, distance=2) 
    # ֱ��ֱ��ǰ��������0.5m, �����и���ѭ����û��ִ�����ʱ���ǲ����������������
    #go_straight_x_until(cmd_vel_pub, speed=0.1, distance=0.7) 
    turn_left(cmd_vel_pub, speed=0.5) # ��ת90��
    # ������ڿ�����Ϊ����ԭ�������£����ŵ�ͼ��y�᷽��ǰ��2m
    # go_straight_y(cmd_vel_pub, speed=0.5, distance=1.5) 

	# ֱ��ֱ��ǰ��������0.5m, �����и���ѭ����û��ִ�����ʱ���ǲ����������������
    # go_straight_y_until(cmd_vel_pub, speed=0.1, distance=0.5) 
    # turn_right(cmd_vel_pub, speed=0.5) # ��ת90��


if __name__ == '__main__':
    try:
        # ROS�ڵ��ʼ��
        rospy.init_node('cmd_vel_test', anonymous=True)
        cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=100)
        main(cmd_vel_pub)
    except rospy.ROSInterruptException:
        pass
