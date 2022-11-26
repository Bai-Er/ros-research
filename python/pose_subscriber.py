import rospy
from geometry_msgs import Pose   # 可能待改

def poseInfoCallback(msg):
    rospy.loginfo("---------Subcribe Pose Info-------------")
    rospy.loginfo("---------Robot_Pos-x: %d, Robot_Pos-y: %d, Robot_Pos-z: %d\n", msg.position.x, msg.position.y, msg.position.z)

def pose_subscriber():
	# ROS节点初始化
    rospy.init_node('pose_subscriber', anonymous=True)

	# 创建一个Subscriber，订阅名为/person_info的topic，注册回调函数personInfoCallback
    rospy.Subscriber("/mobile_base/sensors/imu_data", Pose, poseInfoCallback)

	# 循环等待回调函数
    rospy.spin()

if __name__ == '__main__':
    pose_subscriber()