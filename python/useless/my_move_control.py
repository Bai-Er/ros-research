import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped  # PoseWithCovarianceStamped还不确定


class MyMoveControl(Object):

    def __init__(self):  # 初始化该类
        self.set_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=5) # 创建一个Publisher

    
    def callback(self, msg):  # 接受move_base 发送的话题/cmd_vel
 
        cmd_twist_rotation =  msg.angular.z #
        cmd_twist_x  = msg.linear.x 
        cmd_twist_y =  msg.linear.y #这个一般为0
        
        # 将twist消息转化为左右轮各自的期望速度
        wheelspeed = self.odom_to_speed(cmd_twist_x, cmd_twist_y,cmd_twist_rotation)        
        print 'msg:', msg                #打印得到的twist消息
        print wheelspeed                 #打印转化后的速度 
        
        # 蓝牙串口发送到DSP  wheelspeed[0]左轮速度， wheelspeed[1]右轮速度        
        # self.blue_tooth_send([wheelspeed[0], self.speed_kp, self.speed_ki,  wheelspeed[1]])  

 
    def odom_to_speed(self, cmd_twist_x =0, cmd_twist_y=0,cmd_twist_rotation=0):
        '一般情况下，linear_y = 0 所以只需关注twist.linear.x 和 twist.angle.z的转换'
        #这部分本来还有一段，关于twist.linear.y不为0时，如何转化的程序，Lz自己写的，实际可运行，但是不知道是否正确，所以这里删掉了。
        
        cent_speed = cmd_twist_x        # 前进的速度，即两轮的中心速度
        #将 指定的转速twist.angular.z 转化为左右轮的差速
        yawrate2 = self.yawrate_to_speed(cmd_twist_rotation)   
        
        Lwheelspeed = cent_speed - yawrate2/2
        Rwheelspeed = cent_speed + yawrate2/2
        
        return Lwheelspeed, Rwheelspeed
 
    def yawrate_to_speed(self, yawrate):
        if yawrate > 0:
            theta_to_speed = 0.0077 #右转系数
        else:
            theta_to_speed = 0.0076  #左转系数
        
        #yawrate ：rad/s *0.02表示 20ms内应该转多少弧度，/0.0076是把 要转的弧度转化为左右轮速度差    
        x = (yawrate * 0.02) / theta_to_speed   
        return   x

    def listener(self):
        rospy.init_node('cmd_vel_listener')
        rospy.Subscriber("/cmd_vel", Twist, callback)#/cmd_vel
        rospy.spin()


    def send_to_robot(self, left_speed, right_speed):  # 把期望的速度发送给机器人底盘控制程序 



if __name__ == "__main__":

    rospy.init_node('cmd_vel_listener')
    rospy.Subscriber("/cmd_vel", Twist, callback)  # /cmd_vel

    test1 = MyMoveControl()


