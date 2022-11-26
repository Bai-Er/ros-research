#! /usr/bin/env python
# encoding: utf-8

"""
date:2022/10/26
author:Wang.Zairan
detail:测试定点导航及人为转弯
"""

import rospy
import actionlib  # 用于控制机器人动作的包
import move_base
from actionlib_msgs.msg import * 
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import Imu   # 可能待改
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf_conversions import transformations
from math import pi
from launch_demo import launch_demo

class NavigationControl(Object):

    # 初始化函数，仅执行一次
    def __init__(self):

        """订阅2个传感器信息"""
        # 创建一个Subscriber，订阅名为/mobile_base/sensors/imu_data的topic，注册回调函数imuInfoCallback
        # rospy.Subscriber("/mobile_base/sensors/imu_data", Imu, imuInfoCallback)

        # 创建一个Subscriber，订阅名为odom的topic，注册回调函数odomInfoCallback
        rospy.Subscriber("odom", Odometry, odomInfoCallback)

        """创建一个Publisher话题信息: 设置初始位置"""
        set_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=5)
    
    def set_init(self, init_pose):
        # 设置初始位置
        rospy.loginfo(f"set init pose{init_pose}...")
        r = rospy.Rate(1)
        r.sleep()

        # self.set_pose([0, 0, 0])
        self.set_pose(init_pose)

        rospy.loginfo("finish init pose...")


    # 订阅传感器信息的回调函数
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
        # rospy.loginfo("----odom odom_quat: %0.6f\n", msg.pose.pose.orientation) # 暂时没法打印

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
        # 旋转坐标系的值
        rospy.loginfo("---------Imu orient-x: %0.6f, orient-y: %0.6f, orient-z: %0.6f, orient-w: %0.6f\n", msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        # 三个角速度值
        rospy.loginfo("---------Imu agr_ve-x: %0.6f, agr_ve-y: %0.6f, agr_ve-z: %0.6f\n", msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z)
        # 三个线速度值
        rospy.loginfo("---------Imu lin_ac-x: %0.6f, lin_ac-y: %0.6f, lin_ac-z: %0.6f\n", msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z)

    def set_pose(self, p): # 设置
        if self.move_base is None:
            return False

        x, y, th = p

        # 定义一个数据类型为PoseWithCovarianceStamped变量
        pose = PoseWithCovarianceStamped()  # 印有协方差的姿势
        pose.header.stamp = rospy.Time.now()  #时间戳
        pose.header.frame_id = 'map'  # id戳
        pose.pose.pose.position.x = x  # x值
        pose.pose.pose.position.y = y  # y值
        q = transformations.quaternion_from_euler(0.0, 0.0, th/180.0*pi)  # 生成旋转变换关系
        pose.pose.pose.orientation.x = q[0]
        pose.pose.pose.orientation.y = q[1]
        pose.pose.pose.orientation.z = q[2]
        pose.pose.pose.orientation.w = q[3]

        # 发布该话题数据，实现初始化小车的位置
        set_pose_pub.publish(pose)  

        return True

    def _done_cb(self, status, result):
        """
        两个参数分别是目标值和结果值
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

        # 欧拉角转换为4元数
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
    """初始化结点"""
    rospy.init_node('navigation_demo', anonymous=True)
    
    """运行相关功能包"""
    # launch可执行文件，并执行
    launch_nav = launch_demo(["roslaunch", "turn_on_dlrobot_robot", "navigation.launch"])  # 启动
    launch_nav.launch()

    r = rospy.Rate(0.2)
    r.sleep()

    nav = NavigationControl()
    nav.set_init([0, 0, 90])

    

if __name__ == "__main__":

    main()
