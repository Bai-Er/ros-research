import rospy
from geometry_msgs import Pose   # ���ܴ���

def poseInfoCallback(msg):
    rospy.loginfo("---------Subcribe Pose Info-------------")
    rospy.loginfo("---------Robot_Pos-x: %d, Robot_Pos-y: %d, Robot_Pos-z: %d\n", msg.position.x, msg.position.y, msg.position.z)

def pose_subscriber():
	# ROS�ڵ��ʼ��
    rospy.init_node('pose_subscriber', anonymous=True)

	# ����һ��Subscriber��������Ϊ/person_info��topic��ע��ص�����personInfoCallback
    rospy.Subscriber("/mobile_base/sensors/imu_data", Pose, poseInfoCallback)

	# ѭ���ȴ��ص�����
    rospy.spin()

if __name__ == '__main__':
    pose_subscriber()