#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn
import math
import random

class MultiFollowerTurtle:
    def __init__(self):
        # 1. 初始化节点（必须最先执行）
        # 使用 anonymous=True 确保节点名唯一
        rospy.init_node('follower_node', anonymous=True)

        # 2. 获取参数（使用私有命名空间 ~）
        self.follower_name = rospy.get_param('~follower_name', 'turtle2')
        self.leader_name = rospy.get_param('~leader_name', 'turtle1')
        
        # 为了防止生成冲突，给不同的乌龟分配不同的初始位置
        default_x = 2.0 if self.follower_name == 'turtle2' else 1.0
        default_y = 2.0 if self.follower_name == 'turtle2' else 1.0
        spawn_x = rospy.get_param('~spawn_x', default_x)
        spawn_y = rospy.get_param('~spawn_y', default_y)

        # 3. 核心控制参数
        self.Kp_linear = 1.0       # 线速度比例增益
        self.Kp_angular = 4.0      # 角速度比例增益
        self.target_distance = 1.0 # 跟随距离

        # 4. 自动生成乌龟逻辑
        self.spawn_turtle(spawn_x, spawn_y)

        # 5. 状态变量
        self.leader_pose = Pose()
        self.follower_pose = Pose()
        self.has_leader_pose = False
        self.has_self_pose = False

        # 6. 订阅与发布
        rospy.Subscriber('/%s/pose' % self.leader_name, Pose, self.leader_pose_callback)
        rospy.Subscriber('/%s/pose' % self.follower_name, Pose, self.follower_pose_callback)
        self.cmd_pub = rospy.Publisher('/%s/cmd_vel' % self.follower_name, Twist, queue_size=10)

        self.rate = rospy.Rate(20)

    def spawn_turtle(self, x, y):
        """安全生成乌龟，带有重试和延迟"""
        rospy.wait_for_service('spawn')
        try:
            # 如果是 turtle3，稍微等一下，让 turtle2 先生成
            if self.follower_name == 'turtle3':
                rospy.sleep(0.5)
            
            spawner = rospy.ServiceProxy('spawn', Spawn)
            spawner(x, y, 0.0, self.follower_name)
            rospy.loginfo("成功生成: %s" % self.follower_name)
        except rospy.ServiceException as e:
            rospy.logwarn("%s 生成跳过（可能已存在）" % self.follower_name)

    def leader_pose_callback(self, data):
        self.leader_pose = data
        self.has_leader_pose = True

    def follower_pose_callback(self, data):
        self.follower_pose = data
        self.has_self_pose = True

    def run(self):
        rospy.loginfo("%s 开始监听并跟随 %s..." % (self.follower_name, self.leader_name))
        while not rospy.is_shutdown():
            # 只有获取到双方位置后才计算
            if self.has_leader_pose and self.has_self_pose:
                dx = self.leader_pose.x - self.follower_pose.x
                dy = self.leader_pose.y - self.follower_pose.y
                distance = math.sqrt(dx**2 + dy**2)
                target_angle = math.atan2(dy, dx)

                cmd = Twist()
                if distance > self.target_distance:
                    # 线速度：距离偏差 * Kp
                    cmd.linear.x = self.Kp_linear * (distance - self.target_distance)
                    
                    # 角速度：角度偏差 * Kp（带归一化）
                    angle_error = target_angle - self.follower_pose.theta
                    while angle_error > math.pi: angle_error -= 2 * math.pi
                    while angle_error < -math.pi: angle_error += 2 * math.pi
                    cmd.angular.z = self.Kp_angular * angle_error
                
                self.cmd_pub.publish(cmd)
            
            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = MultiFollowerTurtle()
        node.run()
    except rospy.ROSInterruptException:
        pass
