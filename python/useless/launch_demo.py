import subprocess
import rospy
import rosnode

"""
使用python运行launch文件
"""

class launch_demo:
    def __init__(self, cmd=None):
        self.cmd = cmd

    def launch(self):
        self.child = subprocess.Popen(self.cmd)
        return True

    def shutdown(self):
        self.child.terminate()
        self.child.wait()
        return True

if __name__ == "__main__":
    rospy.init_node('launch_demo',anonymous=True)

    launch_nav = launch_demo(["roslaunch", "pibot_simulator", "nav.launch"])

    launch_nav.launch()

    r = rospy.Rate(0.2)
    r.sleep()
