#! /usr/bin/env python
# encoding: utf-8

import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from turn_on_dlrobot_robot.msg import PointsDeal
from sensor_msgs.msg import Imu
import numpy as np
np.set_printoptions(threshold = np.inf)
import time
import matplotlib.pyplot as plt


class PointsDeal(object):
	def __init__(self):
		# self.sub = rospy.Subscriber("/point_cloud_raw", PointCloud2, self.callback, queue_size=5)
		self.pub = rospy.Publisher("/points_distance", Imu, queue_size=10)
		self.distance_txt_path = "/home/dlrobot/dlrobot_robot/src/turn_on_dlrobot_robot/distance.txt"
		self.zero_points_txt_path = "/home/dlrobot/dlrobot_robot/src/turn_on_dlrobot_robot/zeros_points.txt"
		self.Note_distance = open(self.distance_txt_path, mode='w+')
		self.Note_zero_points = open(self.zero_points_txt_path, mode='w+')

	def main(self):
		while True:
			msg = rospy.wait_for_message("/point_cloud_raw", PointCloud2, timeout=None)
			assert isinstance(msg, PointCloud2)
			points = point_cloud2.read_points_list(msg, field_names=("x", "y", "z"))
			points_degree_zero = self.get_degree_zeros(points) # get 0 degree points
			distance_list = self.points_zero_degree_deal(points_degree_zero, pattern="front") # deal 0 degree points 
			self.publish_distance(self.pub, distance_list) # publish 0 degree points
			self.write_ditance(distance_list)
        	# self.write_points_zero_degree(points_degree_zero)
		

	def get_degree_zeros(self, points):
		points = np.array(points)
		# print(points)
		points_z = np.array(points[:, 2])
		points_0 = np.zeros_like(points_z)
			
		allowwd_error = 3e-2  # thershold control	
		indices = np.where(abs(points_z - points_0) <= allowwd_error)	
		points_degree_zero = points[indices]

		print("{}/{}".format(points_degree_zero.shape[0], points.shape[0]))
		print("\n---------------------------------------\n")
		return points_degree_zero

	def points_zero_degree_deal(self, points_degree_zero, pattern=None, min_angle=None, max_angle=None):
		points_zero_degree_x = points_degree_zero[:, 0]
		points_zero_degree_y = points_degree_zero[:, 1]
		if pattern=="front":
			indices = np.where((points_zero_degree_y < 0) & (-0.3 < points_zero_degree_x) &(points_zero_degree_x <0.3))
		elif pattern=="left":
			indices = np.where((points_zero_degree_x > 0) & (-0.5 < points_zero_degree_y) &(points_zero_degree_y <0.5))
		elif pattern=="right":
			indices = np.where((points_zero_degree_x < 0)& (-0.5 < points_zero_degree_y) & (points_zero_degree_y <0.5))
		else:
			indices = np.where((points_zero_degree_y > 0)&(-0.5 < points_zero_degree_x) & (points_zero_degree_x <0.5))


		points_zero_degree_x = points_zero_degree_x[indices]
		points_zero_degree_y = points_zero_degree_y[indices]

		x_mean = np.mean(points_zero_degree_x)
		x_std = np.std(points_zero_degree_x)
		y_mean = np.mean(points_zero_degree_y)
		y_std = np.std(points_zero_degree_y)
		return [x_mean, x_std, y_mean, y_std]    

	
	    
	def draw_scatter(self, points_degree_zero):
		
		x_value = points_degree_zero[:, 0]
		#print(x_value)
		y_value = points_degree_zero[:, 1]
		#print(y_value)
		plt.scatter(x_value,y_value)
		plt.show()
		time.sleep(10)

	def publish_distance(self, pub, distance_list):
		
		Imu_Data_Pub = Imu()
		Imu_Data_Pub.header.stamp = rospy.Time.now() #当前时间
		Imu_Data_Pub.header.frame_id = "points_deal"
		Imu_Data_Pub.orientation.x = distance_list[0] # x mean
		Imu_Data_Pub.orientation.y = distance_list[1] # x std
		Imu_Data_Pub.orientation.z = distance_list[2] # y mean
		Imu_Data_Pub.orientation.w = distance_list[3] # x std


		Imu_Data_Pub.orientation_covariance = None
		Imu_Data_Pub.angular_velocity.x = 0 
		Imu_Data_Pub.angular_velocity.y = 0
		Imu_Data_Pub.angular_velocity.z = 0
		Imu_Data_Pub.angular_velocity_covariance = None
		Imu_Data_Pub.linear_acceleration.x = 0 
		Imu_Data_Pub.linear_acceleration.y = 0
		Imu_Data_Pub.linear_acceleration.z = 0
		Imu_Data_Pub.linear_acceleration_covariance = None
		pub.publish(Imu_Data_Pub)
		print(distance_list)
		"""
		Data_Pub = PointsDeal()
		Data_Pub.front_mean = distance_list[0] # x mean
		Data_Pub.front_std= distance_list[1] # x std
		Data_Pub.left_mean = distance_list[2] # y mean
		Data_Pub.left_std = distance_list[3] # x std
		Data_Pub.back_mean =None
		Data_Pub.right_mean = None
		Data_Pub.back_std = None
		Data_Pub.right_std = None

		pub.publish(Data_Pub)
		print(distance_list)
		"""

	def write_points_zero_degree(self, points_zero_degree):
		self.Note_zero_points.writelines(str(points_zero_degree)+"\n")
		self.Note_zero_points.flush()
		# self.Note_zero_points.close()

	def write_ditance(self, distance_data):
		self.Note_distance.writelines(str(distance_data)+"\n")
		self.Note_distance.flush()
		# self.Note_distance.close()

if __name__ =='__main__':
    rospy.init_node("pointcloud_subscriber")
    deal = PointsDeal()
    deal.main()
