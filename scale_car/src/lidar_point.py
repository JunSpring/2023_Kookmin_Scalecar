#!/usr/bin/env python
#-*- coding:utf-8 -*-

import os
import rospy
import cv2
import numpy as np
from std_msgs.msg import Int32
from sensor_msgs.msg import CompressedImage
from webot_examples.msg import lidar_msg
from cv_bridge import CvBridge
import message_filters

class PointReceive():
    def __init__(self):
        rospy.Subscriber("/usb_cam/image_rect_color/compressed", CompressedImage, self.Callback)
        rospy.Subscriber("/lidar_pub", lidar_msg, self.state_Callback) # bag 파일이랑 노드 이름이 같아서 lidar 코드 publish 노드 이름 이걸로 임시로 바꿈
        self.lidarX = 0
        self.lidarY = 0
    
    def state_Callback(self, data):
        # 수정해야 할 부분 (나머지는 안건드려도 될 듯) / 코드 실행할 때 오류 뜨면 opencv 버전이 4.2.0 인지 확인 
        self.lidarX = int(data.ywaypoint * 350) + 320 # 좌표 임의로 바꿈, 영상 안에 point 안나오면 좌표 값 x, y 둘 중 하나가 (-)일 것임
        self.lidarY = int(-45 * (data.xwaypoint ** 2) + 150 * data.xwaypoint + 480)
        # self.lidarY = int(-data.xwaypoint * 250 / 8) *  # 좌표 임의로 바꿈
 
    def Callback(self, data):

        bridge=CvBridge()
        image=bridge.compressed_imgmsg_to_cv2(data,"bgr8")

        self.point(image)

    def point(self, image):
        cv2.circle(image, (self.lidarX, self.lidarY), 10, (0, 0, 255), -1) 
        cv2.putText(image, f"({self.lidarX}, {self.lidarY})", (self.lidarX + 30, self.lidarY), cv2.FONT_HERSHEY_SCRIPT_SIMPLEX, 0.7, (0, 0, 255))
        rospy.loginfo("%d %d", self.lidarX, self.lidarY)
        cv2.imshow("image", image)
        cv2.waitKey(1)

def run():
    rospy.init_node("lidar_point")
    new = PointReceive()
    rospy.spin()

if __name__=="__main__":
    try:
        run()
    except rospy.ROSInterruptException:
        pass