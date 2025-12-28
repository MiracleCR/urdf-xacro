#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist
import random
class LeaderTurtle:
    def __init__(self):
        rospy.init_node('leader_turtle', anonymous=True)
        # 发布器：控制 leader 运动
        self.cmd_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        # 定时器：每 2 秒随机改变一次方向
        rospy.Timer(rospy.Duration(2.0), self.random_move)
        rospy.loginfo("Leader Turtle 启动成功！")

    def random_move(self, event):
        """随机生成运动指令"""
        twist = Twist()
        # 随机线速度和角速度
        twist.linear.x = random.uniform(0.3, 1.5) # 线速度范围
        twist.angular.z = random.uniform(-1.0, 1.0) # 角速度范围
        self.cmd_pub.publish(twist)
        rospy.loginfo("Leader - 线速度: %.2f, 角速度: %.2f" % (twist.linear.x, twist.angular.z))
if __name__ == '__main__':
    try:
        leader = LeaderTurtle()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass