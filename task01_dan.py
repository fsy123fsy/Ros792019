#!/usr/bin/env python
# - coding: utf-8 -*-
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class LineFollower:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/cam", Image, self.camera_callback)
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    def otsu_threshold(self, image):
        # 转换为灰度图像
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # 使用大津法计算最佳阈值
        _, binary_image = cv2.threshold(gray_image, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        return binary_image

    def camera_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
            return

        # 使用大津法进行二值化
        binary_image = self.otsu_threshold(cv_image)

        # 截取图像底部区域用于巡线
        height, width = binary_image.shape
        crop_img = binary_image[int(height * 0.8):height, 0:width]

        # 计算质心
        M = cv2.moments(crop_img)
        if M['m00'] > 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            error_x = cx - width // 2
            twist = Twist()
            twist.linear.x = 0.5
            twist.angular.z = float(error_x) / 105
            self.cmd_vel_pub.publish(twist)
        else:
            rospy.loginfo("Line not detected!")

        # 显示图像
        cv2.imshow("Binary Image", binary_image)
        cv2.waitKey(1)

def main():
    rospy.init_node('line_follower', anonymous=True)
    line_follower = LineFollower()
    rospy.spin()

if __name__ == '__main__':
    main()