#! /usr/bin/env python
# encoding: utf-8

"""
date:2022/10/26
author:Wang.Zairan
detail:���Զ��㵼������Ϊת��
"""

import rospy
import actionlib  # ���ڿ��ƻ����˶����İ�
import move_base
from actionlib_msgs.msg import * 
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import Imu   # ���ܴ���
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf_conversions import transformations
from math import pi
from launch_demo import launch_demo

class NavigationControl(Object):

    # ��ʼ����������ִ��һ��
    def __init__(self):

        """����2����������Ϣ"""
        # ����һ��Subscriber��������Ϊ/mobile_base/sensors/imu_data��topic��ע��ص�����imuInfoCallback
        # rospy.Subscriber("/mobile_base/sensors/imu_data", Imu, imuInfoCallback)

        # ����һ��Subscriber��������Ϊodom��topic��ע��ص�����odomInfoCallback
        rospy.Subscriber("odom", Odometry, odomInfoCallback)

        """����һ��Publisher������Ϣ: ���ó�ʼλ��"""
        set_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=5)
    
    def set_init(self, init_pose):
        # ���ó�ʼλ��
        rospy.loginfo(f"set init pose{init_pose}...")
        r = rospy.Rate(1)
        r.sleep()

        # self.set_pose([0, 0, 0])
        self.set_pose(init_pose)

        rospy.loginfo("finish init pose...")


    # ���Ĵ�������Ϣ�Ļص�����
    def poseInfoCallback(self):
        rospy.loginfo("---------Subcribe Pose Info-------------")
        rospy.loginfo("---------Robot_Pos-x: %d, Robot_Pos-y: %d, Robot_Pos-z: %d\n", msg.position.x, msg.position.y, msg.position.z)
        robot_pos_x = msg.position.x
        robot_pos_y = msg.position.y
        robot_pos_z = msg.position.z


    def odomInfoCallback(self, msg):
        rospy.loginfo("Subcribe odom Info: time:%s  name:%s", msg.header.stamp, msg.header.frame_id)
        rospy.loginfo("----odom robot_pos-x: %0.6f\n", msg.pose.pose.position.x)
        rospy.loginfo("----odom robot_pos-y: %0.6f\n", msg.pose.pose.position.y)
        rospy.loginfo("----odom robot_pos-z: %0.6f\n", msg.pose.pose.position.z)
        # rospy.loginfo("----odom odom_quat: %0.6f\n", msg.pose.pose.orientation) # ��ʱû����ӡ

        rospy.loginfo("----odom robot_vel-x: %0.6f\n", msg.twist.twist.linear.x)
        rospy.loginfo("----odom robot_vel-y: %0.6f\n", msg.twist.twist.linear.y)
        rospy.loginfo("----odom robot_ang-z: %0.6f\n", msg.twist.twist.angular.z)

        robot_pos_x = msg.pose.pose.position.x
        robot_pos_x = msg.pose.pose.position.x
        robot_pos_x = msg.pose.pose.position.x

        robot_vel_x = msg.twist.twist.linear.x
        robot_vel_y = msg.twist.twist.linear.y
        robot_ang_z = msg.twist.twist.angular.z
        
        # rospy.loginfo("----odom-pose-covariance: %d\n", msg.pose.covariance)   
        # rospy.loginfo("----odom-twist-covariance: %d\n", msg.twist.covariance)


    def imuInfoCallback(self, msg):
        rospy.loginfo("Subcribe Imu Info: time:%s  name:%s", msg.header.stamp, msg.header.frame_id)
        # ��ת����ϵ��ֵ
        rospy.loginfo("---------Imu orient-x: %0.6f, orient-y: %0.6f, orient-z: %0.6f, orient-w: %0.6f\n", msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        # �������ٶ�ֵ
        rospy.loginfo("---------Imu agr_ve-x: %0.6f, agr_ve-y: %0.6f, agr_ve-z: %0.6f\n", msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z)
        # �������ٶ�ֵ
        rospy.loginfo("---------Imu lin_ac-x: %0.6f, lin_ac-y: %0.6f, lin_ac-z: %0.6f\n", msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z)

    def set_pose(self, p): # ����
        if self.move_base is None:
            return False

        x, y, th = p

        # ����һ����������ΪPoseWithCovarianceStamped����
        pose = PoseWithCovarianceStamped()  # ӡ��Э���������
        pose.header.stamp = rospy.Time.now()  #ʱ���
        pose.header.frame_id = 'map'  # id��
        pose.pose.pose.position.x = x  # xֵ
        pose.pose.pose.position.y = y  # yֵ
        q = transformations.quaternion_from_euler(0.0, 0.0, th/180.0*pi)  # ������ת�任��ϵ
        pose.pose.pose.orientation.x = q[0]
        pose.pose.pose.orientation.y = q[1]
        pose.pose.pose.orientation.z = q[2]
        pose.pose.pose.orientation.w = q[3]

        # �����û������ݣ�ʵ�ֳ�ʼ��С����λ��
        set_pose_pub.publish(pose)  

        return True

    def _done_cb(self, status, result):
        """
        ���������ֱ���Ŀ��ֵ�ͽ��ֵ
        """
        rospy.loginfo("navigation done! status:%d result:%s"%(status, result))

    def _active_cb(self):
        rospy.loginfo("[Navi] navigation has be actived")

    def _feedback_cb(self, feedback):
        rospy.loginfo("[Navi] navigation feedback\r\n%s"%feedback)

    def goto(self, p):
        goal = MoveBaseGoal()

        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = p[0]
        goal.target_pose.pose.position.y = p[1]

        # ŷ����ת��Ϊ4Ԫ��
        q = transformations.quaternion_from_euler(0.0, 0.0, p[2]/180.0*pi)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        move_base.send_goal(goal, self._done_cb, self._active_cb, self._feedback_cb)
        return True

    def cancel(self):
        self.move_base.cancel_all_goals()
        return True

    

def main():
    """��ʼ�����"""
    rospy.init_node('navigation_demo', anonymous=True)
    
    """������ع��ܰ�"""
    # launch��ִ���ļ�����ִ��
    launch_nav = launch_demo(["roslaunch", "turn_on_dlrobot_robot", "navigation.launch"])  # ����
    launch_nav.launch()

    r = rospy.Rate(0.2)
    r.sleep()

    nav = NavigationControl()
    nav.set_init([0, 0, 90])

    

if __name__ == "__main__":

    main()
