import rospy
from nav_msgs.msg import Odometry   # ���ܴ���

def odomInfoCallback(msg):
    rospy.loginfo("Subcribe odom Info: time:%s  name:%s", msg.header.stamp, msg.header.frame_id)
    rospy.loginfo("----odom robot_pos-x: %0.6f\n", msg.pose.pose.position.x)
    rospy.loginfo("----odom robot_pos-y: %0.6f\n", msg.pose.pose.position.y)
    rospy.loginfo("----odom robot_pos-z: %0.6f\n", msg.pose.pose.position.z)
    # rospy.loginfo("----odom odom_quat: %0.6f\n", msg.pose.pose.orientation) # ��ʱû����ӡ

    rospy.loginfo("----odom robot_vel-x: %0.6f\n", msg.twist.twist.linear.x)
    rospy.loginfo("----odom robot_vel-y: %0.6f\n", msg.twist.twist.linear.y)
    rospy.loginfo("----odom robot_ang-z: %0.6f\n", msg.twist.twist.angular.z)

    # rospy.loginfo("----odom-pose-covariance: %d\n", msg.pose.covariance)   
    # rospy.loginfo("----odom-twist-covariance: %d\n", msg.twist.covariance)

def odom_subscriber():
	# ROS�ڵ��ʼ��
    rospy.init_node('odom_subscriber', anonymous=True)

	# ����һ��Subscriber��������Ϊ/person_info��topic��ע��ص�����personInfoCallback
    rospy.Subscriber("odom", Odometry, odomInfoCallback)

	# ѭ���ȴ��ص�����
    rospy.spin()

if __name__ == '__main__':
    odom_subscriber()