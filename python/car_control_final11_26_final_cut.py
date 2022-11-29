#! /usr/bin/env python
# encoding: utf-8

"""
Last updated time: 2022.11.26����
Detail: 2022.11.26ȥ���ӳ�����󱣴�����´���
���и��ļ�, ������������launch�ļ�:
    roslaunch ge_detect_and_run.launch
ע�⣺���ļ����car_function.pyʹ��
"""

import rospy
import time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Pose   # ���ܴ���
import math   # pi
import numpy as np

def Center_detect_11(cmd_vel_pub, road_width=1.1):
    """
    У��������ʻ�ķ���
    """
    threshold = 0.4
    _, _, left_distance, _, right_distance, _, _, _ = Distance_listener(True) # ��ȡ��ǰ���״���ֵ
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
    # �����ת����ת��z>0
    if math.isnan(right_distance):
       cmd_vel_msg.angular.z = - 0.32
    elif math.isnan(left_distance):
        cmd_vel_msg.angular.z = 0.32  

    elif abs(left_distance - right_distance) < road_width / 20: # ��һ��ʱ���ڲ�������
        print("policy-20")
        cmd_vel_msg.angular.z = 0
        
    elif abs(left_distance - right_distance) < road_width / 10: # ��΢��һ��ƫ��
        print("policy-10")
        kp = 0.49
        cmd_vel_msg.angular.z = (left_distance - right_distance) * kp # ���ڱ���ϵ��
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
   #    cmd_vel_msg.angular.z = (left_distance - right_distance) * kp # ���ڱ���ϵ��

   # elif abs(left_distance - right_distance) < road_width / 2:
   #     kp = 1
   #     cmd_vel_msg.angular.z = (left_distance - right_distance) * kp # ���ڱ���ϵ��

    go_rotate = 10
    while go_rotate:
        cmd_vel_pub.publish(cmd_vel_msg)  
        go_rotate -= 1
	# print(go_rotate)
        rate.sleep()
    
    return False


def Center_detect_12(cmd_vel_pub, road_width=1.2):
    """
    У��������ʻ�ķ�����
    """
    threshold = 0.4
    _, _, left_distance, _, right_distance, _, _, _ = Distance_listener(True) # ��ȡ��ǰ���״���ֵ
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
    # �����ת����ת��z>0
    if math.isnan(right_distance):
       cmd_vel_msg.angular.z = - 0.32
    elif math.isnan(left_distance):
        cmd_vel_msg.angular.z = 0.32  

    elif abs(left_distance - right_distance) < road_width / 18: # ��һ��ʱ���ڲ�������
        print("policy-20")
        cmd_vel_msg.angular.z = 0
        
    elif abs(left_distance - right_distance) < road_width / 10: # ��΢��һ��ƫ��
        print("policy-10")
        kp = 0.47
        cmd_vel_msg.angular.z = (left_distance - right_distance) * kp # ���ڱ���ϵ��
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
   #    cmd_vel_msg.angular.z = (left_distance - right_distance) * kp # ���ڱ���ϵ��

   # elif abs(left_distance - right_distance) < road_width / 2:
   #     kp = 1
   #     cmd_vel_msg.angular.z = (left_distance - right_distance) * kp # ���ڱ���ϵ��

    go_rotate = 10
    while go_rotate:
        cmd_vel_pub.publish(cmd_vel_msg)  
        go_rotate -= 1
	# print(go_rotate)
        rate.sleep()
    
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
        go_straight = int ((distance / speed) * send_rate - 1) # 
        
        while go_straight:
            cmd_vel_pub.publish(cmd_vel_msg)
            go_straight -= 1
            # print(go_straight)
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
def turn_left(cmd_vel_pub, speed, add=10, rotate_assert=None):
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
        go_straight = int (math.pi / (2 * speed) * send_rate + add) # �����2�Լ��޸�, ����У��Ħ����
        print(go_straight)
        while go_straight:
            cmd_vel_pub.publish(cmd_vel_msg)
            rate.sleep()
            go_straight -= 1
        print(go_straight)
    return True

"""д���˻�û�в����ܲ�����"""
def turn_right(cmd_vel_pub, speed, add=13, rotate_assert=None):
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
        go_straight = int(-1 * math.pi/(2 * speed) * send_rate + int(add)) # �����2�Լ��޸�, ����У��Ħ����
        while go_straight:
            cmd_vel_pub.publish(cmd_vel_msg)
            rate.sleep()
            go_straight -= 1
            print(go_straight)
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
    end_time = time.time()
    while end_time - start_time < 2*1000:
        cmd_vel_pub.publish(cmd_vel_msg)
        rate.sleep()
    end_time = time.time()


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


def main(cmd_vel_pub):

    # five road
    front_go = 30  # need to test
    while front_go:
        go_straight_x(cmd_vel_pub, 0.7, distance=2) 
    	# ֱ��ֱ��ǰ��������0.5m, �����и���ѭ����û��ִ�����ʱ���ǲ����������������
        Center_detect_11(cmd_vel_pub)
        front_go -= 1
        print(front_go)

    go_straight_x(cmd_vel_pub, 0.7, distance=1.2)
    go_straight_x(cmd_vel_pub, 0.7, distance=2) 
    go_straight_x(cmd_vel_pub, 0.7, distance=2) 
                                      
    go_straight_x_until(cmd_vel_pub, speed=0.1, distance=1.4) 
    turn_left(cmd_vel_pub, speed=0.5) # ��ת90��
    go_straight_x(cmd_vel_pub, 0.7, distance=2.7)
    turn_left(cmd_vel_pub, speed=0.5) # ��ת90��
    go_straight_x(cmd_vel_pub, 0.7, distance=3)
    stop(cmd_vel_pub, time=10)
    
    # four
    front_go = 13
    while front_go:
        go_straight_x(cmd_vel_pub, 0.7, distance=1.5) 
	    # ֱ��ֱ��ǰ��������0.5m, �����и���ѭ����û��ִ�����ʱ���ǲ����������������
        Center_detect_12(cmd_vel_pub, road_width = 1.2)
        front_go -= 1
        print(front_go)

    
    go_straight_x(cmd_vel_pub, 0.7, distance=1.2)
    #------
    go_straight_x_until(cmd_vel_pub, speed=0.1, distance=0.7)  # 0.7
    turn_right(cmd_vel_pub, speed= - 0.5, add=15) # ��ת90��
    go_straight_x(cmd_vel_pub, 0.7, distance=2.6)
    turn_right(cmd_vel_pub, speed= - 0.5, add=13) # ��ת90��
    go_straight_x(cmd_vel_pub, 0.7, distance=2) 
    stop(cmd_vel_pub, time=10)
    print("----end circle----")

    # three
    front_go = 106
    while front_go:
        go_straight_x(cmd_vel_pub, 0.7, distance=0.5) 
	   # ֱ��ֱ��ǰ��������0.5m, �����и���ѭ����û��ִ�����ʱ���ǲ����������������
        Center_detect_11(cmd_vel_pub)
        front_go -= 1
        print(front_go)
    print("----end circle----")

    go_straight_x_until(cmd_vel_pub, speed=0.1, distance=1.5)
    turn_left(cmd_vel_pub, speed=0.5, add=8) # ��ת90��
    go_straight_x(cmd_vel_pub, 0.7, distance=2.6)
    turn_left(cmd_vel_pub, speed=0.5, add=9) # ��ת90��
    go_straight_x(cmd_vel_pub, 0.7, distance=1.5)

    # two
    front_go = 120
    while front_go:
        go_straight_x(cmd_vel_pub, 0.7, distance=0.4) 
	    # ֱ��ֱ��ǰ��������0.5m, �����и���ѭ����û��ִ�����ʱ���ǲ����������������
        Center_detect_12(cmd_vel_pub, road_width = 1.2)
        front_go -= 1
        print(front_go)

    go_straight_x_until(cmd_vel_pub, speed=0.1, distance=0.7)
    turn_right(cmd_vel_pub, speed=-0.5, add=15) # ��ת90��
    go_straight_x(cmd_vel_pub, 0.7, distance=2.6)
    turn_right(cmd_vel_pub, speed=-0.5, add=13) # ��ת90��
    go_straight_x(cmd_vel_pub, 0.7, distance=2)
    print("----end circle----"+front_go)

	
    # one
    front_go = 58
    while True:
        go_straight_x(cmd_vel_pub, 0.7, distance=1) 
	    # ֱ��ֱ��ǰ��������0.5m, �����и���ѭ����û��ִ�����ʱ���ǲ����������������
        Center_detect_11(cmd_vel_pub)
        front_go -= 1
        print(front_go)
    print("----end circle----")
    print("end")
    
    

if __name__ == '__main__':
    try:
        # ROS�ڵ��ʼ��
        rospy.init_node('cmd_vel_test', anonymous=True)
        cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=100)
        main(cmd_vel_pub)
    except rospy.ROSInterruptException:
        pass
