#! /usr/bin/env python
# encoding: utf-8

"""
运行该文件, 必须运行以下launch文件:
roslaunch turn_on_dlrobot_robot navigation.launch 
或者 roslaunch turn_on_dlrobot_robot turn_on_dlrobot_robot.launch 
roslaunch lslidar_driver start.launch
roslaunch pointcloud_to_laserscan pointcloud_scan.launch
"""

import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from turn_on_dlrobot_robot.msg import PointsDeal
from sensor_msgs.msg import Imu
import numpy as np
np.set_printoptions(threshold = np.inf)
import time
import math
import matplotlib.pyplot as plt


class PointsDeal(object):
	def __init__(self):
		# self.sub = rospy.Subscriber("/point_cloud_raw", PointCloud2, self.callback, queue_size=5)
		self.pub = rospy.Publisher("/points_distance", Imu, queue_size=10) # init publisher
		# self.distance_txt_path = "/home/dlrobot/dlrobot_robot/src/turn_on_dlrobot_robot/distance.txt"
		# self.zero_points_txt_path = "/home/dlrobot/dlrobot_robot/src/turn_on_dlrobot_robot/zeros_points.txt"
		# self.Note_distance = open(self.distance_txt_path, mode='w+')
		# self.Note_zero_points = open(self.zero_points_txt_path, mode='w+')
		# self.zero_points_txt_path = "/home/dlrobot/dlrobot_robot/src/turn_on_dlrobot_robot/zeros_points_deal.txt"
		# self.zero_points_deal = open(self.zero_points_txt_path, mode='w+')
		# self.points_txt =  "/home/dlrobot/dlrobot_robot/src/turn_on_dlrobot_robot/points.txt"
		# self.points = open(self.points_txt, mode='w+')

	def main(self):
		while True:
			msg = rospy.wait_for_message("/point_cloud_raw", PointCloud2, timeout=None)
			assert isinstance(msg, PointCloud2)
			points = point_cloud2.read_points_list(msg, field_names=("x", "y", "z"))
			#points_y = point_cloud2.read_points_list(msg, field_names=("y"))
			#points_z = point_cloud2.read_points_list(msg, field_names=("z"))
			#print(points)
			#print(points_array)
			#self.write_points(points_array)
			points_degree_zero= self.get_degree_zeros(points) # get 0 degree points
			distance_list= self.points_zero_degree_deal(points_degree_zero, pattern="three") # deal 0 degree points 
			self.publish_distance(self.pub, distance_list) # publish 0 degree points
			#self.write_ditance(distance_list)
			#self.write_points_zero_degree_deal([points_zero_degree_x, points_zero_degree_y])
        		#self.write_points_zero_degree(points_degree_zero)
		

	def get_degree_zeros(self, points):
		points_array = np.array(points, dtype='float32')

		points_z = np.array(points_array[:, 2])
		points_0 = np.zeros_like(points_z)
			
		allowwd_error = 20e-2  # thershold control	
		# indices = np.where(abs(points_z - points_0) <= allowwd_error)	
		indices = np.where((points_z - points_0  > 0) & (points_z - points_0 <= allowwd_error))
		points_degree_zero = points_array[indices]

		# print("{}/{}".format(points_degree_zero.shape[0], points_array.shape[0]))
		print("\n---------------------------------------\n")
		return points_degree_zero

	def points_zero_degree_deal(self, points_degree_zero, pattern="left"):
		"""
		points_degree_zero : 符合要求的0度数据
		pattern = "front": 模式选择
		return: front_mean, front_std, left_mean, left_std, right_mean, right_std, back_mean, back_std
		"""
		points_zero_degree_x = points_degree_zero[:, 0]
		points_zero_degree_y = points_degree_zero[:, 1]

		if len(points_zero_degree_x) == 0 & len(points_zero_degree_y) == 0:
			return [100, 100, 100, 100, 100, 100, 100, 100] 

		if pattern=="front":
			indices = np.where((points_zero_degree_y > 0) & (-0.3 < points_zero_degree_x) & (points_zero_degree_x < 0.3))
			if len(indices[0]) == 0:
			 	return [100, 100, 100, 100, 100, 100, 100, 100] 
 			# print(indices)
			# points_zero_degree_x = points_zero_degree_x[indices]
            		# print(points_zero_degree_x.shape)
			else:
				points_zero_degree_front = points_zero_degree_y[indices]
				front_mean = np.mean(points_zero_degree_front, axis=0)
				front_std = np.std(points_zero_degree_front, axis=0)
				# if math.isnan(front_mean) and math.isnan(front_std): 
				# 	front_mean, front_std = 100, 100	
				return [front_mean, front_std, 100, 100, 100, 100, 100, 100]

		elif pattern=="left":
			indices = np.where((points_zero_degree_x > -0.8) & (points_zero_degree_x < 0) & (-0.3 < points_zero_degree_y) &(points_zero_degree_y <0.7))
			if len(indices[0]) == 0:
			 	return [100, 100, 100, 100, 100, 100, 100, 100] 
			else:
				points_zero_degree_left = points_zero_degree_x[indices]
				# points_zero_degree_left = np.array(points_zero_degree_left, dtype='int32')
				left_mean = np.median(points_zero_degree_left, axis=0) 
				# left_mean = np.argmax(left_mean) / 10 * -1
				# left_std = np.std(points_zero_degree_left, axis=0)
				# if math.isnan(left_mean) and math.isnan(left_std): 
				# 	left_mean, left_std = 100, 100	
				return [100, 100, left_mean, 100, 100, 100, 100, 100]

		elif pattern=="right":
			indices = np.where((points_zero_degree_x < 0.8) & (points_zero_degree_x > 0) & (-0.3 < points_zero_degree_y) & (points_zero_degree_y < 0.7))
			if len(indices[0]) == 0:
			 	return [100, 100, 100, 100, 100, 100, 100, 100] 
			else:
				points_zero_degree_right = points_zero_degree_x[indices]
				right_mean = np.median(points_zero_degree_right, axis=0)
				# right_std = np.std(points_zero_degree_right, axis=0)
				# if math.isnan(right_mean) and math.isnan(right_std): 
				# 	right_mean, right_std = 100, 100	
				return [100, 100, 100, 100, right_mean, 100, 100, 100]

		elif pattern == "back": 
			indices = np.where((points_zero_degree_y < 0) & (-0.5 < points_zero_degree_x) & (points_zero_degree_x <0.5))
			if len(indices[0]) == 0:
			 	return [100, 100, 100, 100, 100, 100, 100, 100] 
			else:
				points_zero_degree_back = points_zero_degree_y[indices]
				back_mean = np.mean(points_zero_degree_back, axis=0)
				back_std = np.std(points_zero_degree_back, axis=0)
				# if math.isnan(back_mean) and math.isnan(back_std): 
				# 	back_mean, back_std = 100, 100	
				return [100, 100, 100, 100, 100, 100, back_mean, back_std]

		elif pattern == "three":
			indices_left = np.where((points_zero_degree_x > -0.8) & (points_zero_degree_x < 0) & (-0.3 < points_zero_degree_y) & (points_zero_degree_y < 0.7))
			indices_right = np.where((points_zero_degree_x < 0.8) & (points_zero_degree_x > 0) & (-0.3 < points_zero_degree_y) & (points_zero_degree_y < 0.7))
			indices_front = np.where((points_zero_degree_y > 0) & (-0.3 < points_zero_degree_x) & (points_zero_degree_x <0.3))
			points_zero_degree_left = points_zero_degree_x[indices_left]
			points_zero_degree_front = points_zero_degree_y[indices_front]
			points_zero_degree_right = points_zero_degree_x[indices_right]
			left_mean = np.median(points_zero_degree_left, axis=0) 
			# left_std = np.std(points_zero_degree_left, axis=0)
			right_mean = np.median(points_zero_degree_right, axis=0)
			# right_std = np.std(points_zero_degree_right, axis=0)
			front_mean = np.mean(points_zero_degree_front, axis=0)
			front_std = np.std(points_zero_degree_front, axis=0)
			# if math.isnan(left_mean) and math.isnan(left_std) and math.isnan(right_mean) and math.isnan(right_std) and math.isnan(front_mean) and math.isnan(front_std):
			# 	left_mean, left_std, right_mean, right_std ,front_mean, front_std= 100, 100, 100, 100, 100, 100			
			return [front_mean, front_std, left_mean, 100, right_mean, 100 ,100, 100]


	def publish_distance(self, pub, distance_list):
		
		Imu_Data_Pub = Imu()
		Imu_Data_Pub.header.stamp = rospy.Time.now() #当前时间
		Imu_Data_Pub.header.frame_id = "points_deal"
		Imu_Data_Pub.orientation.x = distance_list[0] # front mean
		Imu_Data_Pub.orientation.y = distance_list[1] # front std
		Imu_Data_Pub.orientation.z = distance_list[2] # left mean
		Imu_Data_Pub.orientation.w = distance_list[3] # left std


		Imu_Data_Pub.orientation_covariance = [0,0,0,0,0,0,0,0,0]
		Imu_Data_Pub.angular_velocity.x = distance_list[4]  # right mean
		Imu_Data_Pub.angular_velocity.y = distance_list[5]  # right std
		Imu_Data_Pub.angular_velocity.z = 0
		Imu_Data_Pub.angular_velocity_covariance = [0,0,0,0,0,0,0,0,0]
		Imu_Data_Pub.linear_acceleration.x = distance_list[6]  # back mean
		Imu_Data_Pub.linear_acceleration.y = distance_list[7]  # back std
		Imu_Data_Pub.linear_acceleration.z = 0
		Imu_Data_Pub.linear_acceleration_covariance = [0,0,0,0,0,0,0,0,0]
		pub.publish(Imu_Data_Pub)
		print(distance_list)

"""
	def write_points(self, points):
		self.points.writelines(str(points)+"\n")
		self.points.flush()
		# self.Note_zero_points.close()

	def write_points_zero_degree(self, points_zero_degree):
		self.Note_zero_points.writelines(str(points_zero_degree)+"\n")
		self.Note_zero_points.flush()
		# self.Note_zero_points.close()
	def write_points_zero_degree_deal(self, points_zero_degree_deal):
		self.zero_points_deal.writelines(str(points_zero_degree_deal)+"\n")
		self.zero_points_deal.flush()
		# self.Note_zero_points.close()

	def write_ditance(self, distance_data):
		self.Note_distance.writelines(str(distance_data)+"\n")
		self.Note_distance.flush()
		# self.Note_distance.close()
"""

if __name__ =='__main__':
    rospy.init_node("pointcloud_subscriber")
    deal = PointsDeal()
    deal.main()
