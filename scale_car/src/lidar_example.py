#! /usr/bin/env python
#-*- coding: utf-8 -*-

# ------------------------ import ------------------------
from re import S
from xmlrpc.client import NOT_WELLFORMED_ERROR
import rospy
from obstacle_detector.msg import Obstacles
from webot_examples.msg import lidar_msg
from std_msgs.msg import Float32
import math

# ------------------------ 전역변수 ------------------------
# 이전 state 변수
prev_state = None

# 기본 변수들
start = None # LiDAR 최초 검출 시간기록용 변수
state = 0 # 미션 상태 변수
ready = False # circles가 존재 시 True로 변환되는 변수

# 표지판 변수
sign = None # 차량과 표지판 사이의 거리 변수

# mission1 변수
mission1_running = False # mission1을 수행중인지 판단하는 변수
mission1_start = None # mission1 시작시간기록용 변수
mission1_prev_time = 0

# mission2 변수
mission2_count = 0 # 객체가 왼쪽으로 이동하는 횟수를 저장하는 변수
mission2_prev_data = None # 객체의 이전 x좌표를 저장하는 변수
mission2_start = None # mission2 시작시간기록용 변수
mission2_prev_time = 0

# mission4 변수
mission4_count = 0 # mission4 publish rate 횟수 기록용 변수
mission4_state = False # mission4가 진행중인지 기록하기 위한 변수
mission4_start = None # mission4 시작시간기록용 변수
mission4_prev_time = 0

# ------------------------ class ------------------------
class LidarReceiver():
    def __init__(self):
        self.xwaypoint = 0 # waypoint x좌표
        self.ywaypoint = 0 # waypoint y좌표

        rospy.loginfo("LiDAR Receiver Object is Created")

        # publisher
        self.pub = rospy.Publisher("/lidar_pub", lidar_msg, queue_size=10)
        """
        Publish content
        int32 state (차량 미션 상태)
        float32 xwaypoint (라바콘 waypoint x좌표)
        float32 ywaypoint (라바콘 waypoint y좌표)
        """

        # subscriber
        rospy.Subscriber("raw_obstacles", Obstacles, self.obstacles_callback)
        rospy.Subscriber("/sign_pub", Float32, self.sign_callback)

        # ros가 실행되는 동안 publish_data 함수 반복실행
        while not rospy.is_shutdown():
            self.publish_data()

    # 표지판 subscribe callback 함수
    def sign_callback(self, _data):
        # global 변수
        global sign

        sign = _data.data

    # 객체 검출 subscribe callback 함수
    def obstacles_callback(self, _data):
        # global 변수
        global ready
        global sign
        global mission1_running
        global mission2_count

        circles = _data.circles
        
        # 표지판 검출 시 라이다 상황을 고려하지 않고 mission1 진행
        if sign:
            mission1_running = True

        if mission1_running:
            mission2_count = 0

            self.mission1()

        # 표지판 비검출 시와 circles 검출 시 나머지 mission 진행
        elif circles:
            ready = True
        
        if ready:
            self.state_ready(circles)

    # 객체 검출 시 짧은 시간 저속주행으로 추가 객체 감지를 진행하는 함수
    def state_ready(self, circles):
        # global 변수
        global start
        global state

        if (not start) and (not state):
            start = rospy.Time.now().to_sec()
            state = 5
        else:
            if rospy.Time.now().to_sec() - start < 2:
                state = 5

                # 저속주행시 mission2 가능성을 모색
                self.mission2(False, circles)
            else:
                self.select_mission_number(circles)

    # 추가 객체 감지를 마친 후 mission number를 결정하는 함수
    def select_mission_number(self, circles):
        # global 변수
        global state
        global mission2_count

        # mission2 동적장애물
        if mission2_count > 5:
            self.mission2(True)

        # mission3 라바콘
        elif state == 3 or len(circles) >= 3:
            mission2_count = 0

            if len(circles) >= 3:
                self.mission3(True, circles)
            elif len(circles) == 0:
                self.mission3(False)

        # mission4 정적장애물
        else:
            mission2_count = 0
            
            self.mission4(circles)

    # mission1 어린이보호구역 함수
    def mission1(self):
        # global 변수
        global state
        global sign
        global mission1_start
        global mission1_running
        global mission1_prev_time

        # mission1 start 직후
        if not mission1_start:
            mission1_start = rospy.Time.now().to_sec()
            state = 1
        
        # mission start 직후의 이후
        else:
            time = rospy.Time.now().to_sec() - mission1_start
            if time < 15:
                state = 1
                if time > mission1_prev_time:
                    rospy.loginfo('어린이 보호구역 %d',int(time))
                    mission1_prev_time += 1
            else:
                # 변수 초기화
                self.end_mission()
                mission1_start = None
                mission1_running = False
                mission1_prev_time = 0
                sign = None

    # mission2 동적장애물 함수
    def mission2(self, mission_start, circles = None):
        # global 변수
        global state
        global mission2_count
        global mission2_prev_data
        global mission2_start
        global mission2_prev_time

        closest_circle = None
        min_distance = 100
        
        # 미션 시작 전 mission2인지 판단하기 위한 코드
        if not mission_start:
            if circles:
                # 가장 가까운 circle 판단 코드
                for circle in circles:
                    distance = math.sqrt(circle.center.x ** 2 + circle.center.y ** 2) - circle.true_radius
                    if distance < min_distance:
                        closest_circle = circle
                        min_distance = distance

                # 객체의 x좌표가 감소할 때마다 count하는 코드
                if not mission2_prev_data:
                    mission2_prev_data = closest_circle.center.y
                else:
                    # rospy.loginfo(mission2_prev_data - closest_circle.center.y)
                    if mission2_prev_data - closest_circle.center.y > 0.015:
                        
                        mission2_count += 1
                    mission2_prev_data = closest_circle.center.y
                # rate = rospy.Rate(10)

                # for circle in circles:
                #     distance = math.sqrt(circle.center.x ** 2 + circle.center.y ** 2) - circle.true_radius
                #     if distance < min_distance:
                #         closest_circle = circle
                #         min_distance = distance

                # if not mission2_prev_data:
                #     mission2_prev_data = closest_circle.center.y

                # for i in range(15):
                #     rospy.loginfo(i)
                #     rate.sleep()

                # if mission2_prev_data - closest_circle.center.y > 0.3:
                #     mission2_count = 20

        # mission2가 맞다면 실행하는 코드
        else:
            # mission2 start 직후
            if not mission2_start:
                mission2_start = rospy.Time.now().to_sec()
                state = 2

            # mission2 start 직후의 이후
            else:
                time = rospy.Time.now().to_sec() - mission2_start
                if time < 9:
                    if time > mission2_prev_data:
                        rospy.loginfo("동적장애물 %d", int(time))
                        mission2_prev_time += 1
                else:
                    # 변수 초기화
                    self.end_mission()
                    mission2_count = 0
                    mission2_prev_data = None
                    mission2_start = None
                    mission2_prev_time = 0

    # mission3 라바콘
    def mission3(self, mission_state, circles = None):
        # global 변수
        global state

        # 가장 가까운 두 circle
        circle1 = None
        circle2 = None
        circle1_error = True
        circle2_error = True
        error = False
        min_distance = 100

        if mission_state:
            state = 3

            if circles:
                # 가장 가까운 두 circle을 찾는 코드
                for circle in circles:
                    distance = math.sqrt(circle.center.x ** 2 + circle.center.y ** 2) - circle.true_radius
                    if distance < min_distance and circle.center.y > -0.1: # !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                        circle1 = circle
                        min_distance = distance
                        circle1_error = False

                if not circle1_error:
                    circles.remove(circle1)

                min_distance = 100
                for circle in circles:
                    distance = math.sqrt(circle.center.x ** 2 + circle.center.y ** 2) - circle.true_radius
                    try:
                        if distance < min_distance and circle.center.y < circle1.center.y - 0.45:
                            circle2 = circle
                            min_distance = distance
                            circle2_error = False
                    except:
                        if distance < min_distance and circle.center.y < 0.3:
                            circle2 = circle
                            min_distance = distance
                            circle2_error = False
 
                # try:
                #     str = f"{circle1.center.y}\t{-1*circle1.center.x}\n{circle2.center.y}\t{-1*circle2.center.x}\n"
                #     rospy.loginfo(str)
                # except:
                #     pass

                if not circle1_error and not circle2_error and math.sqrt((circle1.center.x - circle2.center.x) ** 2 + (circle1.center.y - circle2.center.y) ** 2) > 0.7:
                    circle_error = False
                else:
                    circle_error = True

                if circle_error:
                    if circle1_error:
                        self.xwaypoint = -1
                        self.ywaypoint = 1
                    elif circle2_error:
                        self.xwaypoint = -1
                        self.ywaypoint = -1
                    elif not error:
                        self.ywaypoint = -1 * self.ywaypoint
                        error = True
                    else:
                        if not circle_error:
                            error = False

                # 두 circle의 좌표의 평균값을 계산하여 저장
                else:
                    self.xwaypoint = (circle1.center.x + circle2.center.x) / 2
                    self.ywaypoint = (circle1.center.y + circle2.center.y) / 2

        else:
            # 변수 초기화
            self.end_mission()
            self.xwaypoint = 0
            self.ywaypoint = 0  

    # mission4 정적장애물 함수
    def mission4(self, circles):
        # global 변수
        global state
        global mission4_count
        global mission4_state
        global mission4_start
        global mission4_prev_time

        detected_circle = None

        rate = rospy.Rate(10)

        if circles:
            # 가장 가까운 circle 판단 코드
            for circle in circles:
                if -0.1 <= circle.center.y <= 0.1:
                    detected_circle = circle

        if not mission4_state:
            try:
                if detected_circle:
                    state = 4
                    mission4_state = True
                    mission4_start = rospy.Time.now().to_sec()
                else:
                    self.end_mission()
                    mission4_count = 0
                    mission4_state = False
                    mission4_start = None
            except:
                self.end_mission()
                mission4_count = 0
                mission4_state = False
                mission4_start = None

        if mission4_state:
            if mission4_count <= 5:
                rate.sleep()
                mission4_count += 1
                return
            else:
                state = 0

        if mission4_start and rospy.Time.now().to_sec() - mission4_start > 1.75:
            # 변수 초기화
            self.end_mission()
            mission4_count = 0
            mission4_state = False
            mission4_start = None
            mission4_prev_time = 0

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
        global sign

        # publish data 대입
        publishing_data = lidar_msg()
        publishing_data.state = state
        publishing_data.xwaypoint = self.xwaypoint
        publishing_data.ywaypoint = self.ywaypoint


        # publish
        self.pub.publish(publishing_data)

        # rospy.loginfo
        if prev_state != state:
            self.loginfo(state)
        
        # if state == 5:
        #     rospy.loginfo(mission2_count)
        prev_state = state

    def loginfo(self, data):
        if data == 0:
            str = "평시주행"
        elif data == 1:
            str = "어린이 보호구역"
        elif data == 2:
            str = "동적 장애물"
        elif data == 3:
            str = "라바콘"
        elif data == 4:
            str = "정적 장애물"
        elif data == 5:
            str = "판단중"

        rospy.loginfo(str)
        
# ------------------------ run ------------------------
def run():
    rospy.init_node("lidar_example")
    new_class = LidarReceiver()
    rospy.spin()

# ------------------------ __name__ ------------------------
if __name__=='__main__':
    run()