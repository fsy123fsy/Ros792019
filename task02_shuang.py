#!/usr/bin/env python
# - coding: utf-8 -*-
import rospy
import cv2
import numpy as np
import os 
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist


import time

start_time=time.time()

class LineFollower:
    def __init__(self):
        self.bridge = CvBridge()
        # self.pose_sub=rospy.Subscriber("/pose",Pose,self.pose_callback)
        self.image_sub = rospy.Subscriber('/cam', Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.twist = Twist()
        # 调整PID参数
        self.Kp = 0.018
        self.Ki = 0.00001
        self.Kd = 0.00011
        self.error_prev = 0
        self.error_sum = 0
        # 定义预定轨迹，这里简单示例为一条直线
        self.yudingguiji = [(0.02, 2.2), (-2.5, 2.6), (-2.1, 3.5)]
        self.current_waypoint_index = 0
        # 假设图像宽度为640，将物理坐标转换为像素坐标的比例因子
        self.pixel_scale = 640  

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
            return

        # 截取图像的下半部分
        height, width = cv_image.shape[:2]
        cv_image = cv_image[height*2//3:,:]

        processed_image = self.preprocess_image(cv_image)
        warped_image = self.perspective_transform(processed_image)
        # lower_half=warped_image[height*2//3:,:]# 取下半部分图像

        # 检测透视图中间区域是否有白色像素
        mid_width = width *2//3
        mid_height = height *2//3
        middle_region = warped_image[mid_height-15:mid_height+15, mid_width-15:mid_width+15]
        white_pixels = cv2.countNonZero(middle_region)
        has_white_in_middle = white_pixels > 0

        # 寻找双线
        left_line, right_line = self.find_lines(warped_image)

        # 计算中心点偏移
        height, width = warped_image.shape
        center_x = width // 2
        if left_line is not None and right_line is not None:
            target_x = (left_line + right_line) // 2
        elif left_line is not None:
            target_x = left_line + 50
            print("左线丢失，向右偏移-------------------------------------------------------")
        elif right_line is not None:
            target_x = right_line - 45
            print("右线丢失，向左偏移-------------------------------------------------------")
        else:
            target_x = center_x
            print("两线丢失，保持原方向-----------------------------------------------------")

        # 计算与预定轨迹的偏差
        if self.current_waypoint_index < len(self.yudingguiji):
            waypoint_x = int(self.yudingguiji[self.current_waypoint_index][0] * self.pixel_scale) + center_x
            path_error = target_x - waypoint_x
            if abs(target_x - waypoint_x) < 2:  # 如果接近当前路径点，移动到下一个
                self.current_waypoint_index += 1
        else:
            path_error = 0

        error = target_x - center_x
        # error=path_error
        self.error_sum += error
        error_diff = error - self.error_prev
        self.error_prev = error

        # PID控制器
        angular_z = self.Kp * error + self.Ki * self.error_sum + self.Kd * error_diff

        # 如果透视图中间有白色区域，增加向左的转向偏移量
        if has_white_in_middle:
            angular_z += 0.1 # 可以根据实际情况调整偏移量
            print("透视图中间有白色区域，向左转----------------------------------------------")

        print("转角的值为")
        print(angular_z)
        
        # 限制转角值
        now_time=time.time()
        run_time=now_time-start_time
        if 0<run_time<65:
            angular_z = np.clip(angular_z, -0.3, 0.3)
            self.twist.linear.x = 0.18
            self.twist.angular.z = angular_z
            self.cmd_vel_pub.publish(self.twist)
        if 65<=run_time<70:
            self.twist.linear.x=0.0
            self.twist.angular.z=0.3
            self.cmd_vel_pub.publish(self.twist)
        if 70<=run_time< 143:#143
            angular_z = np.clip(angular_z, -0.3, 0.3)
            self.twist.linear.x = 0.18
            self.twist.angular.z = angular_z
            self.cmd_vel_pub.publish(self.twist)
        # if 103<=run_time< 105:#143
        #     angular_z = np.clip(angular_z, -0.3, 0.3)
        #     self.twist.linear.x = 0.19
        #     self.twist.angular.z = 0
        #     self.cmd_vel_pub.publish(self.twist)
        # if 105<=run_time< 143:#143
        #     angular_z = np.clip(angular_z, -0.3, 0.3)
        #     self.twist.linear.x = 0.19
        #     self.twist.angular.z = angular_z
        #     self.cmd_vel_pub.publish(self.twist)
        if   143<=run_time<150:
            print("运动到识别区，接下来走环岛")
            self.twist.linear.x=0.6
            self.twist.angular.z=0
            self.cmd_vel_pub.publish(self.twist)
        if 149<=run_time<152:    
            self.twist.linear.x=0
            self.twist.angular.z=0
            self.cmd_vel_pub.publish(self.twist)
            a=int(input("请选择环岛入口:"))    
            if a==1:
                os.system("python xiesi03.py")
                
            elif a==2:
                self.twist.linear.x=0.7
                self.twist.angular.z=-0.2
                self.cmd_vel_pub.publish(self.twist)
                    
                
                
        
        
        
        
        
        # key = cv2.waitKey(3)
        # if key == ord('w'):  # 按 'w' 键增大Kp
        #     self.Kp += 0.0001
        #     print("增大Kp+0.0001")
        # elif key == ord('s'):  # 按 's' 键减小Kp
        #     self.Kp -= 0.0001
        #     print("减小Kp-0.0001")
        # elif key == ord('e'):  # 按 'e' 键增大Ki
        #     self.Ki += 0.000001
        #     print("增大Ki+0.0001")
        # elif key == ord('d'):  # 按 'd' 键减小Ki
        #     self.Ki -= 0.000001
        #     print("减小Ki-0.0001")
        # elif key == ord('r'):  # 按 'r' 键增大Kd
        #     self.Kd += 0.00001
        #     print("增大Kd+0.0001")
        # elif key == ord('f'):  # 按 'f' 键减小Kd
        #     self.Kd -= 0.00001
        #     print("减小Kd-0.0001")


        # 显示处理后的图像
        target_x = int(target_x)
        height = int(height) 
        cv2.line(cv_image, (target_x, int(height * 0.6)), (target_x, height), (0, 255, 0), 2)
        cv2.imshow("Line Follower", cv_image)
        cv2.imshow("Warped Image", warped_image)
        cv2.waitKey(3)

    def preprocess_image(self, cv_image):
        # 转换为灰度图像
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        # 高斯模糊以减少噪声
        blurred = cv2.GaussianBlur(gray, (7, 7),3 )
        # 自适应阈值处理
        thresh = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 11, 2)
        kernel = np.ones((5, 5), np.uint8)
        opening = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
        closing = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel)
        return closing
        # return thresh

    def perspective_transform(self, binary_image):
        height, width = binary_image.shape
        # 调整源点和目标点以获得更好的透视变换效果
        src_points = np.float32([
            [width * 0.25, height * 0.7],
            [width * 0.75, height * 0.7],
            [width * 0.1, height],
            [width * 0.9, height]
        ])
        dst_points = np.float32([
            [width * 0.2, 0.3],
            [width * 0.8, 0.3],
            [width * 0.2, height],
            [width * 0.8, height]
        ])
        M = cv2.getPerspectiveTransform(src_points, dst_points)
        warped_image = cv2.warpPerspective(binary_image, M, (width, height))
        return warped_image

    def find_lines(self, warped_image):
        height, width = warped_image.shape
        left_line = None
        right_line = None

        # 直方图统计
        histogram = np.sum(warped_image[height // 2:, :], axis=0)
        midpoint = np.int32(histogram.shape[0] // 2)
        left_base = np.argmax(histogram[:midpoint])
        right_base = np.argmax(histogram[midpoint:]) + midpoint

        # 滑动窗口法寻找线
        nwindows = 9
        window_height = np.int32(height // nwindows)
        leftx_current = left_base
        rightx_current = right_base
        margin = 100
        minpix = 50
        left_lane_inds = []
        right_lane_inds = []

        nonzero = warped_image.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])

        for window in range(nwindows):
            win_y_low = height - (window + 1) * window_height
            win_y_high = height - window * window_height
            win_xleft_low = leftx_current - margin
            win_xleft_high = leftx_current + margin
            win_xright_low = rightx_current - margin
            win_xright_high = rightx_current + margin

            good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                              (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
            good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                               (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]

            left_lane_inds.append(good_left_inds)
            right_lane_inds.append(good_right_inds)

            if len(good_left_inds) > minpix:
                leftx_current = np.int32(np.mean(nonzerox[good_left_inds]))
            if len(good_right_inds) > minpix:
                rightx_current = np.int32(np.mean(nonzerox[good_right_inds]))

        left_lane_inds = np.concatenate(left_lane_inds)
        right_lane_inds = np.concatenate(right_lane_inds)

        if len(left_lane_inds) > 0:
            left_line = np.int32(np.mean(nonzerox[left_lane_inds]))
        if len(right_lane_inds) > 0:
            right_line = np.int32(np.mean(nonzerox[right_lane_inds]))

        return left_line, right_line

if __name__ == '__main__':
    rospy.init_node('line_follower')
    line_follower = LineFollower()
    rospy.spin()