#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist
import math

LINEAR_SPEED = 0.5
ANGULAR_SPEED = 0.3

class UcarControl:
    def __init__(self):
        rospy.init_node("ucar_control")
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)
        rospy.sleep(1)

    def rotate(self, angle, clockwise):
        twist = Twist()
        angular_speed = -ANGULAR_SPEED if clockwise else ANGULAR_SPEED
        twist.angular.z = angular_speed
        duration = abs(angle / angular_speed)
        t0 = rospy.Time.now().to_sec()
        while (rospy.Time.now().to_sec() - t0) < duration:
            self.pub.publish(twist)
            self.rate.sleep()
        twist.angular.z = 0
        self.pub.publish(twist)

    def move_forward(self, distance):
        twist = Twist()
        twist.linear.x = LINEAR_SPEED
        duration = distance / LINEAR_SPEED
        t0 = rospy.Time.now().to_sec()
        while (rospy.Time.now().to_sec() - t0) < duration:
            self.pub.publish(twist)
            self.rate.sleep()
        twist.linear.x = 0
        self.pub.publish(twist)

    def move_in_circle(self, radius, fraction):
        angular_speed = LINEAR_SPEED / radius
        twist = Twist()
        twist.linear.x = LINEAR_SPEED
        twist.angular.z = angular_speed
        angle = 2 * math.pi * fraction
        duration = angle / angular_speed
        t0 = rospy.Time.now().to_sec()
        while (rospy.Time.now().to_sec() - t0) < duration:
            self.pub.publish(twist)
            self.rate.sleep()
        twist.linear.x = 0
        twist.angular.z = 0
        self.pub.publish(twist)

    def run(self):
        print("开始顺时针旋转135度")
        self.rotate(math.radians(145), True)
        print("顺时针旋转135度完成，开始直走2米")
        self.move_forward(2)
        print("直走2米完成，开始逆时针旋转45度")
        self.rotate(math.radians(70), False)
        print("逆时针旋转45度完成，开始直走2米")
        self.move_forward(2)
        print("直走2米完成，停止3秒")
        rospy.sleep(1)
        print("停止结束，开始走半径为1.5的四分之三圆")
        self.move_in_circle(1.5, 0.20)
        print("四分之三圆走完，开始顺时针旋转60度")
        self.rotate(math.radians(80), True)
        print("顺时针旋转60度完成，开始直走")
        self.move_forward(3)
        print("运动结束")

if __name__ == "__main__":
    try:
        ucar_control = UcarControl()
        ucar_control.run()
    except rospy.ROSInterruptException:
        pass