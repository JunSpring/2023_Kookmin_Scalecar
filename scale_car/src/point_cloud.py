#! /usr/bin/env python
#-*- coding: utf-8 -*-

# ------------------------ import ------------------------
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan, PointCloud
from geometry_msgs.msg import Point32
from std_msgs.msg import Float64
from scale_car.msg import center_msg

import math

from enums import StateNum

# ------------------------ 전역변수 ------------------------
# 좌우 ROI Parameter
lateral_roi_param = 0.5
# 앞뒤 ROI Parameter
axial_roi_param = 0

# 영점
origin = (0, 0)

# 미션 상태
state = StateNum.NORMAL_DRIVING

# ------------------------ class ------------------------
class rpScanfReceiver:
    def __init__(self):
        self.lscan = rospy.Subscriber("/scan", LaserScan, self.callback)
        rospy.Subscriber("/center_data", center_msg, self.center_callback)
        self.pc_pub = rospy.Publisher("/pcl", PointCloud, queue_size=5)
 
    def callback(self, data):
        min_angle = data.angle_min
        min_range = data.range_min
        max_range = data.range_max
        angle_increment = data.angle_increment
        PC_data = PointCloud()
        PC_data.header = data.header

        for i, range in enumerate(data.ranges):
            x, y = calc_axis_xy(min_angle + angle_increment * i, range, min_range, max_range)
            if is_data(x, y):
                PC_data.points.append(Point32(x, y, 0))
        self.pc_pub.publish(PC_data)
    
    def center_callback(self, data):
        global lateral_roi_param
        global axial_roi_param
        global state

        state = data.state

def calc_axis_xy(_theta, _distance, _min_range, _max_range):
    if _min_range <= _distance <= _max_range:
        x = np.cos(_theta) * _distance
        y = np.sin(_theta) * _distance
        return (x, y)   
    else:
        return (0, 0)

def is_data(_x, _y):
    global lateral_roi_param
    global axial_roi_param
    global state

    if (_x, _y) == (0, 0):
        return False
    if state == StateNum.RUBBERCON_DRIVING and\
        calc_distance(origin, (_x, _y)) <= 1 and -100 <= calc_angle(origin, (-1*_x, _y)) <= 100:
        return True
    if state != StateNum.RUBBERCON_DRIVING and\
        -0.5 <= -1*_y <= 0.5 and 0.0 <= -1*_x <= 1.5:
        return True
    
    return False

# ------------------------ test ------------------------
def calc_distance(point1, point2):
    x1, y1 = point1
    x2, y2 = point2

    # Calculate the differences in x and y coordinates
    dx = x2 - x1
    dy = y2 - y1

    # Calculate the distance using the Pythagorean theorem
    distance = math.sqrt(dx**2 + dy**2)

    return distance

def calc_angle(point1, point2):
    x1, y1 = point1
    x2, y2 = point2

    # # Lidar X-axis transformation
    # x1, x2 = -x1, -x2

    # Calculate the differences in x and y coordinates
    dx = x2 - x1
    dy = y2 - y1

    # Calculate the angle in radians using the arctan2 function
    angle_rad = math.atan2(dy, dx)

    # Convert the angle from radians to degrees
    angle_deg = math.degrees(angle_rad)

    return angle_deg

# ------------------------ run ------------------------
def run():
    rospy.init_node('scan_py_receiver', anonymous=True)
    lc = rpScanfReceiver()
    rospy.spin()

# ------------------------ __name__ ------------------------
if __name__ == "__main__":
    run()