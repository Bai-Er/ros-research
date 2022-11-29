#! /usr/bin/env python
# encoding: utf-8

import rospy
from sensor_msgs.msg import Imu   # 可能待改

def imuInfoCallback(msg):
    rospy.loginfo("Subcribe Imu Info: time:%s  name:%s", msg.header.stamp, msg.header.frame_id)
    # 旋转坐标系的值
    rospy.loginfo("---------Imu orient-x: %0.6f, orient-y: %0.6f, orient-z: %0.6f, orient-w: %0.6f\n", msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
    # 三个角速度值
    rospy.loginfo("---------Imu agr_ve-x: %0.6f, agr_ve-y: %0.6f, agr_ve-z: %0.6f\n", msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z)
    # 三个线速度值
    rospy.loginfo("---------Imu lin_ac-x: %0.6f, lin_ac-y: %0.6f, lin_ac-z: %0.6f\n", msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z)

def imu_subscriber():
	# ROS节点初始化
    rospy.init_node('imu_subscriber', anonymous=True)

	# 创建一个Subscriber，订阅名为/person_info的topic，注册回调函数personInfoCallback
    rospy.Subscriber("/mobile_base/sensors/imu_data", Imu, imuInfoCallback)

	# 循环等待回调函数
    rospy.spin()

if __name__ == '__main__':
    imu_subscriber()