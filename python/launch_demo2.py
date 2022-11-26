from launch_demo import launch_demo
import rospy

import actionlib  # ���ڿ��ƻ����˶����İ�
from actionlib_msgs.msg import * 
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf_conversions import transformations
from math import pi

class navigation_demo:
    def __init__(self):
        # ����һ�����⣺���峵�ӵĳ�ʼ��̬��������Ϊ/initialpose, messagetypeΪPoseWithCovarianceStamped��������Ϊ5
        self.set_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=5)


        # ����һ������С��Ŀ��λ�õĿͻ��ˣ��ͻ�����move_base, ��������������ΪMoveBaseAction
        # MoveBaseAction��������ֵtarget_pose��
        # ��ǰ�н�ֵbase_position��
        # self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        # �ȴ�60ȷ�����������ڣ������򴴽��ɹ�
        # self.move_base.wait_for_server(rospy.Duration(60))

        # self.move_base_goal = rospy.Publisher('/move_base/goal', MoveBaseAction, queue_size=10)
        # 


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
        self.set_pose_pub.publish(pose)  
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

        self.move_base.send_goal(goal, self._done_cb, self._active_cb, self._feedback_cb)
        return True

    def cancel(self):
        self.move_base.cancel_all_goals()
        return True

    def set_goal(self):
        goal = MoveBaseAction()
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
        self.move_base_goal.publish(goal)
        rospy.loginfo("----set-goal-end----")
        return True



if __name__ == "__main__":
    # ��ʼ���ڵ�
    rospy.init_node('navigation_demo',anonymous=True)

    # launch��ִ���ļ�����ִ��
    launch_nav = launch_demo(["roslaunch", "turn_on_dlrobot_robot", "navigation.launch"])  # ������������̼ư�
    launch_nav.launch()

    r = rospy.Rate(0.2)
    r.sleep()

    rospy.loginfo("set pose...")
    r = rospy.Rate(1)
    r.sleep()

    navi = navigation_demo()
    # navi.set_pose([-0.7, -0.4, 0])

    # rospy.loginfo("goto goal...")
    # r = rospy.Rate(1)
    # r.sleep()
    # navi.goto([0.25, 4, 90])

    # ������Ƿ���ܵ��ͻ�������response
    navi.set
    while not rospy.is_shutdown():
        r.sleep()
