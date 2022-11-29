#! /usr/bin/env python
# encoding: utf-8

import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from test_pkg.msg import point_deal
import numpy as np
import time
import matplotlib.pyplot as plt


class PointsDeal(object):
	def __init__(self):
		# self.sub = rospy.Subscriber("/point_cloud_raw", PointCloud2, self.callback, queue_size=5)
		self.pub = rospy.Publisher("/points_distance", Imu, queue_size=10)
		self.txt_path = "/home/dlrobot/dlrobot_robot/src/turn_on_dlrobot_robot/distance.txt"
		self.Note = open(self.txt_path, mode='w+')

	def main(self):
		while True:
			msg = rospy.wait_for_message("/point_cloud_raw", PointCloud2, timeout=None)
			assert isinstance(msg, PointCloud2)
			points = point_cloud2.read_points_list(msg, field_names=("x", "y", "z", "width"))
			points_degree_zero = self.get_degree_zeros(points) # get 0 degree points
			distance_list = self.points_zero_degree_deal(points_degree_zero) # deal 0 degree points 
			self.publish_distance(self.pub, distance_list) # publish 0 degree points
			# self.write_ditance(self.distance_list)
        	# self.write_points_zero_degree(self.points_degree_zero)
		

	def get_degree_zeros(self, points):
		points = np.array(points)
		print(points)
		points_z = np.array(points[:, 2])
		points_0 = np.zeros_like(points_z)
			
		allowwd_error = 20e-2  # thershold control	
		indices = np.where(abs(points_z - points_0) <= allowwd_error)	
		points_degree_zero = points[indices]

		print("{}/{}".format(points_degree_zero.shape[0], points.shape[0]))
		print("\n---------------------------------------\n")
		return points_degree_zero

	def points_zero_degree_deal(self, pattern, points_degree_zero, min_angle, max_angle):
		points_zero_degree_x = points_degree_zero[:, 0]
		points_zero_degree_y = points_degree_zero[:, 1]
		points_zero_degree_number = points_degree_zero[:, 3]
		assert (0 <= points_zero_degree_number <= 1800).all()
		if pattern=="front":
			indices = np.where(points_zero_degree_number > min_angle//0.3 and  points_zero_degree_number < max_angle//0.3)
		elif pattern=="left":
			indices = np.where(points_zero_degree_number > (min_angle-90)//0.3 and  points_zero_degree_number < (max_angle-90)//0.3)
		elif pattern=="left":
			indices = np.where(points_zero_degree_number > (min_angle+90)//0.3 and  points_zero_degree_number < (max_angle+90)//0.3)
		else:
			indices = np.where(points_zero_degree_number < (min_angle+180)//0.3 < 360//0.3 and  0 < points_zero_degree_number < (max_angle-180)//0.3)


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

	def write_points_zero_degree(self, points_zero_degree):
		self.Note.writelines(str(points_zero_degree))
		self.Note.flush()
		self.Note.close()

	def write_ditance(self, distance_data):
		self.Note.writelines(str(distance_data))
		self.Note.flush()
		self.Note.close()

if __name__ =='__main__':
    rospy.init_node("pointcloud_subscriber")
    deal = PointsDeal()
    deal.main()