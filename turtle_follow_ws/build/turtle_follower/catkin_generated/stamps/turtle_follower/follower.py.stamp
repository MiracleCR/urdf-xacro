#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class FollowerTurtle:
    def __init__(self):
        # 通过私有参数获取名字（便于多实例）和 spawn 位置
        follower_name = rospy.get_param('~follower_name', 'turtle2')
        leader_name = rospy.get_param('~leader_name', 'turtle1')
        node_name = 'follower_turtle_' + follower_name

        rospy.init_node(node_name, anonymous=True)

        self.follower_name = follower_name
        self.leader_name = leader_name

        spawn_x = rospy.get_param('~spawn_x', 5.0)
        spawn_y = rospy.get_param('~spawn_y', 5.0)
        spawn_theta = rospy.get_param('~spawn_theta', 0.0)

        # PID 参数（可通过参数服务器调整）
        self.Kp_linear = rospy.get_param('~Kp_linear', 1.0)
        self.Ki_linear = rospy.get_param('~Ki_linear', 0.1)
        self.Kd_linear = rospy.get_param('~Kd_linear', 0.05)
        self.Kp_angular = rospy.get_param('~Kp_angular', 4.0)
        self.Ki_angular = rospy.get_param('~Ki_angular', 0.2)
        self.Kd_angular = rospy.get_param('~Kd_angular', 0.1)

        # 目标跟随距离
        self.target_distance = rospy.get_param('~target_distance', 1.0)

        # 积分限幅（抗 wind-up）
        self.max_integral_linear = rospy.get_param('~max_integral_linear', 1.0)
        self.max_integral_angular = rospy.get_param('~max_integral_angular', 1.0)

        # 最大速度限制
        self.max_linear_speed = rospy.get_param('~max_linear_speed', 2.0)
        self.max_angular_speed = rospy.get_param('~max_angular_speed', 2.0)

        # 主循环频率
        self.rate_hz = rospy.get_param('~rate_hz', 20)

        # 创建 follower（如果不存在）
        try:
            rospy.wait_for_service('spawn', timeout=5.0)
            from turtlesim.srv import Spawn
            spawner = rospy.ServiceProxy('spawn', Spawn)
            spawner(spawn_x, spawn_y, spawn_theta, self.follower_name)
        except Exception:
            rospy.logwarn("%s 可能已存在或 spawn 服务不可用" % self.follower_name)

        # 订阅器：使用参数化的 topic 名称
        rospy.Subscriber('/{}/pose'.format(self.leader_name), Pose, self.leader_pose_callback)
        rospy.Subscriber('/{}/pose'.format(self.follower_name), Pose, self.follower_pose_callback)

        # 发布器：控制 follower 运动（参数化）
        self.cmd_pub = rospy.Publisher('/{}/cmd_vel'.format(self.follower_name), Twist, queue_size=10)

        # 存储位姿数据
        self.leader_pose = Pose()
        self.follower_pose = Pose()

        # 积分/微分状态
        self.integral_distance = 0.0
        self.integral_angle = 0.0
        self.last_distance_error = 0.0
        self.last_angle_error = 0.0

        # 上次时间，用于计算 dt
        self.last_time = rospy.get_time()

        # 速率对象
        self.rate = rospy.Rate(self.rate_hz)

        rospy.loginfo("Follower '%s' 启动，跟随 leader '%s'。" % (self.follower_name, self.leader_name))

    def leader_pose_callback(self, data):
        """回调函数：更新 leader 位姿"""
        self.leader_pose = data

    def follower_pose_callback(self, data):
        """回调函数：更新 follower 位姿"""
        self.follower_pose = data

    def compute_control(self):
        """计算控制指令（带 PID）"""
        dx = self.leader_pose.x - self.follower_pose.x
        dy = self.leader_pose.y - self.follower_pose.y

        distance = math.sqrt(dx**2 + dy**2)
        target_angle = math.atan2(dy, dx)

        distance_error = distance - self.target_distance
        angle_error = target_angle - self.follower_pose.theta

        # 角度归一化到[-pi, pi]
        while angle_error > math.pi:
            angle_error -= 2 * math.pi
        while angle_error < -math.pi:
            angle_error += 2 * math.pi

        # 计算 dt
        now = rospy.get_time()
        dt = now - self.last_time
        if dt <= 0.0 or dt > 1.0:
            dt = 1.0 / float(self.rate_hz)

        # 积分累加（I）
        self.integral_distance += distance_error * dt
        self.integral_angle += angle_error * dt

        # 积分限幅（anti-windup）
        self.integral_distance = max(-self.max_integral_linear, min(self.max_integral_linear, self.integral_distance))
        self.integral_angle = max(-self.max_integral_angular, min(self.max_integral_angular, self.integral_angle))

        # 微分项（D）
        derivative_distance = (distance_error - self.last_distance_error) / dt
        derivative_angle = (angle_error - self.last_angle_error) / dt

        # PID 控制律
        linear_output = (self.Kp_linear * distance_error +
                         self.Ki_linear * self.integral_distance +
                         self.Kd_linear * derivative_distance)
        angular_output = (self.Kp_angular * angle_error +
                          self.Ki_angular * self.integral_angle +
                          self.Kd_angular * derivative_angle)

        # 更新上次误差和时间
        self.last_distance_error = distance_error
        self.last_angle_error = angle_error
        self.last_time = now

        # 封装为 Twist 并限幅
        cmd = Twist()
        cmd.linear.x = max(-self.max_linear_speed, min(self.max_linear_speed, linear_output))
        cmd.angular.z = max(-self.max_angular_speed, min(self.max_angular_speed, angular_output))
        return cmd

    def run(self):
        """主循环"""
        while not rospy.is_shutdown():
            try:
                cmd = self.compute_control()
                self.cmd_pub.publish(cmd)

                # 调试信息（半秒一次）
                dist_err = math.sqrt((self.leader_pose.x - self.follower_pose.x)**2 +
                                     (self.leader_pose.y - self.follower_pose.y)**2) - self.target_distance
                angle_err = math.atan2(self.leader_pose.y - self.follower_pose.y,
                                       self.leader_pose.x - self.follower_pose.x) - self.follower_pose.theta
                rospy.loginfo_throttle(0.5,
                    "Follower '%s' - dist_err: %.3f, angle_err: %.3f, I_dist: %.3f, I_ang: %.3f" %
                    (self.follower_name, dist_err, angle_err, self.integral_distance, self.integral_angle))
            except Exception as e:
                rospy.logwarn("计算控制指令时出错: %s" % str(e))
            self.rate.sleep()

if __name__ == '__main__':
    try:
        follower = FollowerTurtle()
        follower.run()
    except rospy.ROSInterruptException:
        pass