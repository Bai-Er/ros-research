from launch_demo import launch_demo
import rospy

import actionlib  # 用于控制机器人动作的包
from actionlib_msgs.msg import * 
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf_conversions import transformations
from math import pi

class GoalControl(Object):
     def __init__(self):
        # 发布一个话题：定义车子的初始姿态，话题名为/initialpose, messagetype为PoseWithCovarianceStamped，缓冲区为5
        self.set_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=5)


        # 创建一个发布小车目标位置的客户端，客户端名move_base, 发布的数据类型为MoveBaseAction
        # MoveBaseAction包括发送值target_pose：
        # 当前行进值base_position：
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        # 等待60确定服务器存在，存在则创建成功
        self.move_base.wait_for_server(rospy.Duration(60))


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
        self.set_pose_pub.publish(pose)  
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

        self.move_base.send_goal(goal, self._done_cb, self._active_cb, self._feedback_cb)
        return True

    def cancel(self):
        self.move_base.cancel_all_goals()
        return True

if __name__ == "__main__":
    # 初始化节点
    rospy.init_node('navigation_demo',anonymous=True)

    # launch可执行文件，并执行
    launch_nav = launch_demo(["roslaunch", "turn_on_dlrobot_robot", "navigation.launch"])  # 这是启动了里程计吧
    launch_nav.launch()

    r = rospy.Rate(0.2)
    r.sleep()

    rospy.loginfo("set init pose...")
    r = rospy.Rate(1)
    r.sleep()

    navi = GoalControl()
    navi.set_pose([0, 0, 0]) # 设置小车的起始位置 包括x值 y值 和 th旋转坐标关系

    # 循环判断当前小车是否走到起始初值
    

    # 走到指定目标值
    rospy.loginfo("goto goal[1]...")
    r = rospy.Rate(1)
    r.sleep()
    navi.goto([0.25, 4, 90])

    # 服务端是否接受到客户端请求并response
    while not rospy.is_shutdown():
        r.sleep()