#!/usr/bin/env python
#-*- coding:utf-8 -*-

import os
import rospy
import cv2
from vision_msgs.msg import Detection2DArray
from vision_msgs.msg import ObjectHypothesisWithPose
from fiducial_msgs.msg import FiducialTransformArray
from std_msgs.msg import Float32
from cv_bridge import CvBridge

class SignReceiver():
    def __init__(self):
        rospy.loginfo("Sign Object is Created")
        rospy.Subscriber("/fiducial_transforms", Detection2DArray, self.Callback)
        self.pub = rospy.Publisher("/sign_pub", Float32, queue_size = 10)

    def Callback(self, data):

        msg = data.detections[0].results[0].pose.pose.position.z
        self.pub.publish(msg)
  
        rospy.loginfo(msg)

def run():
    rospy.init_node("sign_node")
    signal = SignReceiver()
    rospy.spin()

if __name__=="__main__":
    try:
        run()
    except rospy.ROSInterruptException:
        pass
