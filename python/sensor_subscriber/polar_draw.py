# encoding: utf-8
"""
绘制雷达的点数据
2022.10.29
Wang.
"""
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

csv_path = "F:\\（汪再冉）毕业设计（基于LiDAR+IMU的猪舍巡检机器人导航算法研究）\\ros_code_final\\data.csv"
df= pd.read_csv(csv_path)
angle = np.array(df['angle'])
distance = np.array(df['distance'])
distance[distance == 'inf' ] = 0
distance = distance.astype('float')  # 必须这样写
angle = angle.astype('float')/180*np.pi  # angle-->rad
print(angle.shape)
print(distance)

area = 100 * distance**2 #面积
ax = plt.subplot(111, projection='polar')

#projection为画图样式，除'polar'外还有'aitoff', 'hammer', 'lambert'等
# c = ax.scatter(theta, r, c=colors, s=area, cmap='cool', alpha=0.75)
c = ax.scatter(angle, distance, color='red', cmap='cool')

#ax.scatter为绘制散点图函数
plt.show()