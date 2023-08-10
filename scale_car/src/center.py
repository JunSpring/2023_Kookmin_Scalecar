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
from enums import signNum

# ------------------------ 전역변수 ------------------------
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

Obstacles_callback_start = False
sign_callback_start = True
Yolo_Objects_callback_start = False

prev_state = 0
state = 0

circles = None

sign_distance = None

yolo = None
yolo_size = None

start = None

# ------------------------ class ------------------------
class Center():
    def __init__(self):
        rospy.loginfo("Center Object is Created")

        # publisher
        self.pub = rospy.Publisher("/center_data", center_msg, queue_size=10)
        """
        Publish content
        int32 state (차량 미션 상태)
        float32 xwaypoint (라바콘 waypoint x좌표)
        float32 ywaypoint (라바콘 waypoint y좌표)
        """

        # subscriber
        rospy.Subscriber("/raw_obstacles", Obstacles, self.Obstacles_callback)
        rospy.Subscriber("/sign_pub", Float32, self.sign_callback)
        rospy.Subscriber("/yolov5_pub", Yolo_Objects, self.Yolo_Objects_callback)

        # ros가 실행되는 동안 publish_data 함수 반복실행
        while not rospy.is_shutdown():
            # self.publish_data()
            pass

    # 객체 검출 subscribe callback 함수
    def Obstacles_callback(self, data):
        global circles

        circles = data.circles

        self.excute_mission()
        # rospy.loginfo("%f",self.nearest_circle_distance())

    # 표지판 subscribe callback 함수
    def sign_callback(self, data):
        # global 변수
        global sign_distance

        sign_distance = data.data

        self.excute_mission()

    def Yolo_Objects_callback(self, data):
        global yolo
        global yolo_size

        max_size = 0
        max_index = None

        for yo in data.yolo_objects:
            size = self.calculate_size(yo.x1, yo.x2, yo.y1, yo.y2)
            if size > max_size:
                max_size = size
                max_index = yo.c

        yolo = max_index
        yolo_size = max_size

    def calculate_size(self, x1, x2, y1, y2):
        width = abs(x2 - x1)
        height = abs(y2 - y1)
        size = math.sqrt(width**2 + height**2)
        return size
    
    def excute_mission(self):
        global state

        if state == StateNum.NORMAL_DRIVING or state is None: 
            self.normal_driving()
            self.select_mission_number()

        else:
            if state == (StateNum.SCHOOL_ZONE_SIGN_RECOGNITION or\
                        StateNum.SCHOOL_ZONE_CROSSING_RECOGNITION or\
                        StateNum.SCHOOL_ZONE_RESTART):
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
        global sign_distance
        global yolo
        global yolo_size

        if sign_distance < 100:
            state = StateNum.SCHOOL_ZONE_SIGN_RECOGNITION

        # elif yolo == YoloNum.DYNAMIC_OBSTACLE and self.nearest_circle_distance() < 100:
        #     state = StateNum.DYNAMIC_OBSTACLE

        # elif False and self.nearest_circle_distance() < 100:
        #     state = StateNum.RUBBERCON_DRIVING

        elif True and self.nearest_circle_distance(-0.1, 0.1) < 1.5:
            state = StateNum.STATIC_OBSTACLE

        else:
            state == StateNum.NORMAL_DRIVING

    # (0, 0) 위치로부터 가장 가까운 원의 중심점까지의 거리를 반환하는 함수
    def nearest_circle_distance(self, left, right):
        global circles

        min_distance = float('inf') # 처음에는 무한대로 설정

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

        print(min_distance)

        return min_distance
    
    def normal_driving(self):
        pass

    # mission1 어린이보호구역 함수
    def school_zone(self):
        global state
        global yolo
        global yolo_size
        global start

        now = rospy.Time.now().to_sec()

        if state == StateNum.SCHOOL_ZONE_SIGN_RECOGNITION:
            if yolo == YoloNum.SCHOOL_ZONE and yolo_size > 100:
                state == StateNum.SCHOOL_ZONE_CROSSING_RECOGNITION

        elif state == StateNum.SCHOOL_ZONE_CROSSING_RECOGNITION:
            if start is None:
                start = now

            elif start and now - start > 5.0:
                start = None
                state = StateNum.SCHOOL_ZONE_RESTART
                yolo = None
                yolo_size = None

        elif state == StateNum.SCHOOL_ZONE_RESTART:
            if yolo == YoloNum.SCHOOL_ZONE and yolo_size > 100:
                state == StateNum.NORMAL_DRIVING

    # mission2 동적장애물 함수
    def dynamic_object(self):
        # global 변수
        global state

        # rospy.loginfo("dynamic_object")

    # mission3 라바콘
    def rubber_cone(self):
        # global 변수
        global state

        # rospy.loginfo("rubber_cone")

    # mission4 정적장애물 함수
    def static_object(self):
        # global 변수
        global state
        global start
        global yolo
        global yolo_size

        now = rospy.Time.now().to_sec()

        if start is None:
            start = now

        elif start and now - start > 1.75:
            start = None
            state = StateNum.NORMAL_DRIVING
            yolo = None
            yolo_size = None

    # 공통되는 변수를 초기화하는 함수
    def end_mission(self):
        # global 변수
        global start
        global state
        global ready

        # 변수 초기화
        start = None
        state = 0
        ready = False

    # publish 함수
    def publish_data(self):
        # global 변수
        global start
        global state
        global prev_state

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
        if data == StateNum.NORMAL_DRIVING:
            str = "normal driving"
        elif data == (StateNum.SCHOOL_ZONE_sign_RECOGNITION or\
                    StateNum.SCHOOL_ZONE_CROSSING_RECOGNITION or\
                    StateNum.SCHOOL_ZONE_RESTART):
            str = "school zone"
        elif data == StateNum.DYNAMIC_OBSTACLE:
            str = "dynamic object"
        elif data == StateNum.RUBBERCON_DRIVING:
            str = "rubber cone"
        elif data == StateNum.STATIC_OBSTACLE:
            str = "static object"

        rospy.loginfo(str)
        
# ------------------------ run ------------------------
def run():
    rospy.init_node("Center_Node")
    new_class = Center()
    rospy.spin()

# ------------------------ __name__ ------------------------
if __name__=='__main__':
    run()