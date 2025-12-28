#!/usr/bin/env python3
import rospy, math
from turtlesim.msg import Pose

class DistMonitor:
    def __init__(self):
        rospy.init_node('distance_monitor', anonymous=True)
        self.t1 = None
        self.t2 = None
        rospy.Subscriber('/turtle1/pose', Pose, self.cb1)
        rospy.Subscriber('/turtle2/pose', Pose, self.cb2)
        self.rate = rospy.Rate(10)  # 10 Hz 输出

    def cb1(self, msg):
        self.t1 = (msg.x, msg.y)

    def cb2(self, msg):
        self.t2 = (msg.x, msg.y)

    def run(self):
        while not rospy.is_shutdown():
            if self.t1 and self.t2:
                d = math.hypot(self.t2[0]-self.t1[0], self.t2[1]-self.t1[1])
                rospy.loginfo("distance = %.6f" % d)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        DistMonitor().run()
    except rospy.ROSInterruptException:
        pass