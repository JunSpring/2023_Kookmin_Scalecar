#! /usr/bin/env python
#-*- coding: utf-8 -*-

# ------------------------ import ------------------------
from re import S
from xmlrpc.client import NOT_WELLFORMED_ERROR
import rospy
import math

from obstacle_detector.msg import Obstacles
from scale_car.msg import center_msg
from scale_car_yolov5.msg import Yolo_Objects
from std_msgs.msg import Float32

from enums import StateNum
from enums import YoloNum

# ------------------------ 전역변수 ------------------------
# conda activate VIT

# # 이전 state 변수
# prev_state = None

# # 기본 변수들
# start = None                # LiDAR 최초 검출 시간기록용 변수
# state = 0                   # 미션 상태 변수
# ready = False               # circles가 존재 시 True로 변환되는 변수

# # 표지판 변수
# sign = None                 # 차량과 표지판 사이의 거리 변수

# # mission1 변수
# mission1_running = False    # mission1을 수행중인지 판단하는 변수
# mission1_start = None       # mission1 시작시간기록용 변수
# mission1_prev_time = 0

# # mission2 변수
# mission2_count = 0          # 객체가 왼쪽으로 이동하는 횟수를 저장하는 변수
# mission2_prev_data = None   # 객체의 이전 x좌표를 저장하는 변수
# mission2_start = None       # mission2 시작시간기록용 변수
# mission2_prev_time = 0

# # mission4 변수
# mission4_count = 0          # mission4 publish rate 횟수 기록용 변수
# mission4_state = False      # mission4가 진행중인지 기록하기 위한 변수
# mission4_start = None       # mission4 시작시간기록용 변수
# mission4_prev_time = 0

prev_state = None
state = StateNum.NORMAL_DRIVING_WITH_YOLO

circles = None

yolo = None
yolo_size = None
yolo_nothing_count = 0

start = None

ready_to_crosswalk = False

# ------------------------ class ------------------------
class Center():
    def __init__(self):
        rospy.loginfo("Center Object is Created")

        # publisher
        self.pub = rospy.Publisher("/center_data", center_msg, queue_size=10)

        # subscriber
        rospy.Subscriber("/raw_obstacles", Obstacles, self.Obstacles_callback)
        rospy.Subscriber("/yolov5_pub", Yolo_Objects, self.Yolo_Objects_callback)

        # ros가 실행되는 동안 publish_data 함수 반복실행
        while not rospy.is_shutdown():
            self.excute_mission()
            self.publish_data()

    # 객체 검출 subscribe callback 함수
    def Obstacles_callback(self, data):
        global circles

        circles = data.circles

    def Yolo_Objects_callback(self, data):
        global state
        global yolo
        global yolo_size
        global yolo_nothing_count

        max_index = None
        max_size = 0

        if data.yolo_objects[0].c == YoloNum.NOTHING:
            yolo_nothing_count += 1

            if yolo_nothing_count >= 6:
                yolo = YoloNum.NOTHING

        else:
            yolo_nothing_count = 0

            for yo in data.yolo_objects:
                size = self.calculate_size(yo.x1, yo.x2, yo.y1, yo.y2)

                if state == StateNum.SCHOOL_ZONE_SIGN_RECOGNITION and yo.c == YoloNum.CROSS_WALK:
                    max_size = size
                    max_index = yo.c
                    break

                if (yo.c == YoloNum.ARUCO_MARKER) and (0.8 <= abs((yo.x2-yo.x1)/(yo.y2-yo.y1)) <= 1.2):
                    max_size = size
                    max_index = yo.c
                    break

                if (yo.c != YoloNum.ARUCO_MARKER) and size > max_size:
                    max_size = size
                    max_index = yo.c

            yolo = max_index
            yolo_size = max_size

        # rospy.loginfo(yolo)

    def calculate_size(self, x1, x2, y1, y2):
        width = abs(x2 - x1)
        height = abs(y2 - y1)
        size = math.sqrt(width**2 + height**2)

        return size
    
    def excute_mission(self):
        global state

        if state == StateNum.NORMAL_DRIVING or state == StateNum.NORMAL_DRIVING_WITH_YOLO or state is None: 
            self.normal_driving()
            self.select_mission_number()

        else:
            if state == StateNum.SCHOOL_ZONE_SIGN_RECOGNITION or\
                state == StateNum.SCHOOL_ZONE_CROSSING_RECOGNITION or\
                state == StateNum.SCHOOL_ZONE_RESTART:
                self.school_zone()
            
            elif state == StateNum.DYNAMIC_OBSTACLE:
                self.dynamic_object()

            elif state == StateNum.RUBBERCON_DRIVING:
                self.rubber_cone()

            elif state == StateNum.STATIC_OBSTACLE:
                self.static_object()

    # 추가 객체 감지를 마친 후 mission number를 결정하는 함수
    def select_mission_number(self):
        # global 변수
        global state
        global yolo

        if yolo == YoloNum.ARUCO_MARKER:
            state = StateNum.SCHOOL_ZONE_SIGN_RECOGNITION

        elif yolo == YoloNum.DYNAMIC_OBSTACLE and yolo_size > 100 and self.nearest_circle_distance(-0.5, 0.5) < 1.0:
            state = StateNum.DYNAMIC_OBSTACLE

        elif yolo == YoloNum.RUBBERCONE and self.nearest_circle_distance(-0.5, 0.5) < 0.5:
            state = StateNum.RUBBERCON_DRIVING

        elif yolo == YoloNum.STATIC_OBSTACLE and self.nearest_circle_distance(-0.1, 0.1) < 1.0:
            state = StateNum.STATIC_OBSTACLE

        elif yolo == YoloNum.NOTHING:
            state = StateNum.NORMAL_DRIVING

        else:
            state = StateNum.NORMAL_DRIVING_WITH_YOLO

    # (0, 0) 위치로부터 가장 가까운 원의 중심점까지의 거리를 반환하는 함수
    def nearest_circle_distance(self, left, right):
        global circles

        min_distance = float('inf') # 처음에는 무한대로 설정

        if circles is not None:
            for circle in circles:
                center_x = circle.center.x
                center_y = circle.center.y

                if not left <= center_y <= right:
                    continue

                # (0, 0) 위치와 중심점 간의 거리 계산
                distance_to_center = math.sqrt(center_x**2 + center_y**2)

                # 현재 원의 중심점과의 거리가 이전 원들과의 거리보다 작으면 업데이트
                if distance_to_center < min_distance:
                    min_distance = distance_to_center

        return min_distance
    
    def normal_driving(self):
        pass

    # mission1 어린이보호구역 함수
    def school_zone(self):
        global state
        global yolo
        global yolo_size
        global start
        global ready_to_crosswalk

        now = rospy.Time.now().to_sec()

        if state == StateNum.SCHOOL_ZONE_SIGN_RECOGNITION:
            if yolo == YoloNum.CROSS_WALK and yolo_size > 300:
                state = StateNum.SCHOOL_ZONE_CROSSING_RECOGNITION
                yolo = None
                yolo_size = None

        elif state == StateNum.SCHOOL_ZONE_CROSSING_RECOGNITION:
            if start is None:
                start = now

            elif now - start > 5.0:
                state = StateNum.SCHOOL_ZONE_RESTART
                yolo = None
                yolo_size = None
                start = None

        elif state == StateNum.SCHOOL_ZONE_RESTART:
            if not ready_to_crosswalk and yolo == YoloNum.ARUCO_MARKER:
                ready_to_crosswalk = True

            elif ready_to_crosswalk and yolo == YoloNum.CROSS_WALK and yolo_size > 300:
                state = StateNum.NORMAL_DRIVING_WITH_YOLO
                yolo = None
                yolo_size = None
                ready_to_crosswalk = False

    # mission2 동적장애물 함수
    def dynamic_object(self):
        # global 변수
        global state
        global yolo
        global yolo_size
        global start

        now = rospy.Time.now().to_sec()

        if start is None:
            start = now

        elif now - start > 10.0:
            state = StateNum.NORMAL_DRIVING_WITH_YOLO
            yolo = None
            yolo_size = None
            start = None

    # mission3 라바콘
    def rubber_cone(self):
        # global 변수
        global state
        global yolo
        global yolo_size

        if self.nearest_circle_distance(-1.0, 1.0) == float('inf'):
            state = StateNum.NORMAL_DRIVING_WITH_YOLO
            yolo = None
            yolo_size = None

    # mission4 정적장애물 함수
    def static_object(self):
        # global 변수
        global state
        global yolo
        global yolo_size
        global start

        now = rospy.Time.now().to_sec()

        if start is None:
            start = now

        elif start and now - start > 1.75:
            state = StateNum.NORMAL_DRIVING_WITH_YOLO
            yolo = None
            yolo_size = None
            start = None

    # publish 함수
    def publish_data(self):
        # global 변수
        global prev_state
        global state

        # publish data 대입
        publishing_data = center_msg()
        publishing_data.state = state

        # publish
        self.pub.publish(publishing_data)

        # rospy.loginfo
        if prev_state != state:
            self.loginfo(state)

        prev_state = state

    def loginfo(self, data):
        if data == StateNum.NORMAL_DRIVING_WITH_YOLO:
            str = "normal driving with yolo"
        elif data == StateNum.NORMAL_DRIVING:
            str = "normal driving"
        elif data == StateNum.SCHOOL_ZONE_SIGN_RECOGNITION:
            str = "school zone sign recognition"
        elif data == StateNum.SCHOOL_ZONE_CROSSING_RECOGNITION:
            str = "school zone crossing recognition"
        elif data == StateNum.SCHOOL_ZONE_RESTART:
            str = "school zone restart"
        elif data == StateNum.DYNAMIC_OBSTACLE:
            str = "dynamic object"
        elif data == StateNum.RUBBERCON_DRIVING:
            str = "rubber cone"
        elif data == StateNum.STATIC_OBSTACLE:
            str = "static object"

        rospy.loginfo(str)
        # rospy.loginfo(yolo_nothing_count)
        
# ------------------------ run ------------------------
def run():
    rospy.init_node("Center_Node")
    center = Center()
    rospy.spin()

# ------------------------ __name__ ------------------------
if __name__=='__main__':
    run()