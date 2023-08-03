#!/usr/bin/env python
#-*- coding:utf-8 -*-

import rospy				
from lane_detection.msg import detected_msg   

def Callback(data):
    rospy.loginfo("x: %f | y: %f", data.xdetected, data.ydetected)
    
def main():
    rospy.init_node("ld_sub", anonymous=False)
    rospy.Subscriber("/lane_pub", detected_msg, Callback)
    rospy.spin()

if __name__ == '__main__':
    main()