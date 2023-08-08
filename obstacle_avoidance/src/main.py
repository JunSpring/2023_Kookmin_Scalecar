#! /usr/bin/env python
#-*- coding: utf-8 -*-

# ------------------------ import ------------------------
import rospy
from obstacle_detector.msg import Obstacles
from obstacle_avoidance.msg import avoid_angle
from webot_examples.msg import lidar_msg

import math         # 수학 계산을 위해 import
import numpy as np  # exp 사용을 위해 import

# import matplotlib.pyplot as plt # 그래프 그리기 위해 import

# ------------------- Global Variables -------------------
# launch file params
angle_range = None  # 라이다 인지 및 제어 각도 범위
gamma = None        # 감마 파라미터
d_max = None        # 라이다 인지 최대거리
sigma_param = None  # 시그마 파라미터

w_robot = 0.20      # 차체 폭
theta_goal = 0.0    # 목표 각도

circles = None      # circles 객체
min_angle = 0.0     # 최종 제어 각도
origin = (0, 0)     # 원점

# ------------------------ class -------------------------
class ObstacleAvoidance:
    def __init__(self):
        # loginfo (객체 생성 출력)
        rospy.loginfo("Obstacle Avoidance is Created")

        # Load Param
        global angle_range
        global d_max
        global gamma
        global sigma_param
        
        angle_range = rospy.get_param("angle_range", 100)
        d_max = rospy.get_param("d_max", 2.0)
        gamma = rospy.get_param("gamma", 0.1)
        sigma_param = rospy.get_param("sigma_param", 50.0)

        # # 그래프 그리기
        # self.fig, self.ax = plt.subplots()
        # self.ln, = plt.plot([], [], 'ro')
        # self.x_data, self.y_data = [] , []
        # self.x_data = list(range(-angle_range, angle_range+1))

        # # Publisher
        self.pub = rospy.Publisher("/avoid_angle", avoid_angle, queue_size=10)

        # Subscriber
        rospy.Subscriber("/raw_obstacles", Obstacles, self.Obstacles_callback)
        # rospy.Subscriber("/lidar_pub", lidar_msg, self.lidar_pub_callback)
        
        # ros가 실행되는 동안 publish_data 함수 반복실행
        while not rospy.is_shutdown():
            self.publish_data()

    # Obstacels Callback Function
    def Obstacles_callback(self, msg):
        # Global Variables
        global circles
        global min_angle

        circles = msg.circles

        # 최종 제어 각도 구하기
        min_angle = self.find_min_f_totoal()

    # # LiDAR Pub Callback Function
    # def lidar_pub_callback(self, msg):
    #     global theta_goal
    #     global origin

    #     coneWaypoint_x = msg.xwaypoint #라바콘 웨이포인트 x좌표
    #     coneWaypoint_y = msg.ywaypoint #라바콘 웨이포인트 y좌표

    #     theta_goal = self.calc_angle(origin, (coneWaypoint_x, coneWaypoint_y))

    # Publish Data Function
    def publish_data(self):
        # Global Variable
        global min_angle

        # publish data 대입
        publishing_data = avoid_angle()
        publishing_data.angle = min_angle

        # publish
        self.pub.publish(publishing_data)

        # loginfo (publish 및 최종 제어 각도 출력)
        rospy.loginfo("publish min angle : %f", min_angle)

    # f_total 함수에서 최솟값을 갖는 각도를 리턴하는 함수
    def find_min_f_totoal(self):
        # Global Variable
        global angle_range

        # -100도에서 100도까지의 Potential Field를 저장할 리스트
        f_total = []

        # 각 각도별로 calc_f_total의 값을 리스트에 저장
        for angle in range(-1*angle_range, angle_range+1):
            value = self.calc_f_total(angle)

            # loginfo (angle에 대한 value 출력)
            # rospy.loginfo("%d : \t%f", angle, value)

            f_total.append(value)

        # # 201개의 인덱스를 생성 (-100부터 100까지)
        # x_values = list(range(-angle_range, angle_range+1))

        # # 그래프 그리기
        # plt.plot(x_values, f_total, label="Data")
        # plt.xlabel("angle in")
        # plt.ylabel("Potential Field")
        # plt.title("ODG-PF")
        # plt.legend()
        # plt.grid(True)

        # 최솟값을 가지는 인덱스(angle)를 리턴
        return f_total.index(min(f_total))-angle_range

    # Calculate f_total
    def calc_f_total(self, theta_i):
        f_rep = self.calc_f_rep(theta_i)
        f_att = self.calc_f_att(theta_i)
        f_total = f_rep + f_att

        return f_total

    # Calculate f_rep
    def calc_f_rep(self, theta_i):
        # Global Variable
        global circles

        f_rep = 0

        for circle in circles:
            theta_k = self.calc_theta_k(circle.center)
            d_k = self.calc_d_k(circle.center)
            f_k = self.calc_f_k(theta_i, theta_k, d_k, circle.true_radius)
            f_rep += f_k

        return f_rep

    # Calculate theta_k
    def calc_theta_k(self, point):
        # Global Variable
        global origin

        return self.calc_angle(origin, (point.x, point.y))

    # Caculate d_k
    def calc_d_k(self, point):
        # Global Variable
        global origin

        return self.calc_distance(origin, (point.x, point.y))

    # Calculate f_k
    def calc_f_k(self, theta_i, theta_k, d_k, radius):
        A_k = self.calc_A_k(d_k)
        sigma_k = self.calc_sigma_k(radius, d_k)

        return A_k * np.exp(-1*((theta_k-theta_i)**2)/(2*(sigma_k**2)))

    # Calculate A_k
    def calc_A_k(self, d_k):
        # Global Variable
        global d_max

        tilde_d_k = d_max - d_k
        A_k = tilde_d_k * np.exp(0.5)

        return A_k 
    
    # Calculate sigma_k
    def calc_sigma_k(self, radius, d_k):
        # Global Variables
        global w_robot
        global sigma_param

        """
        논문의 공식대로 시그마를 대입하면 가우시안 분포가 좁게 분포됨
        넓게 분포되도록 sigma_param을 설정함
        """
        return sigma_param * math.atan2(radius+w_robot, d_k)

    # Calculate f_att
    def calc_f_att(self, theta_i):
        # Global Variables
        global gamma
        global theta_goal

        return gamma * abs(theta_goal - theta_i)

    # Calculate Distance
    def calc_distance(self, point1, point2):
        x1, y1 = point1
        x2, y2 = point2

        # Calculate the differences in x and y coordinates
        dx = x2 - x1
        dy = y2 - y1

        # Calculate the distance using the Pythagorean theorem
        distance = math.sqrt(dx**2 + dy**2)

        return distance

    # Calculate Angle
    def calc_angle(self, point1, point2):
        x1, y1 = point1
        x2, y2 = point2

        # Lidar X-axis transformation
        x1, x2 = -x1, -x2

        # Calculate the differences in x and y coordinates
        dx = x2 - x1
        dy = y2 - y1

        # Calculate the angle in radians using the arctan2 function
        angle_rad = math.atan2(dy, dx)

        # Convert the angle from radians to degrees
        angle_deg = math.degrees(angle_rad)

        return angle_deg

# ------------------------- run --------------------------
def run():
    rospy.init_node("obstacle_avoidance")
    oa = ObstacleAvoidance()
    # plt.show(block=True)
    rospy.spin()

# ----------------------- __name__ ----------------------
if __name__=='__main__':
    run()