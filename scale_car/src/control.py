#! /usr/bin/env python
#-*- coding: utf-8 -*-

import rospy
import math
from ackermann_msgs.msg import AckermannDriveStamped
from lane_detection.msg import lane_msg
from std_msgs.msg import Int64
from scale_car.msg import center_msg

from enums import StateNum

# -------------------------------------------------------------------------------
from obstacle_avoidance.msg import avoid_angle
# -------------------------------------------------------------------------------

class Controller:
    lane = -1  #-1 = 1차선, 1 = 2차선
    mission = StateNum.NORMAL_DRIVING_WITH_YOLO
    
    def __init__(self):
        rospy.Subscriber("/lane_pub", lane_msg, self.lane_callback)
        rospy.Subscriber("/center_data", center_msg, self.missionNum)
        rospy.Subscriber("/avoid_angle", avoid_angle, self.avoid_angle_callback)

        self.lane_pub=rospy.Publisher('/whatLane',Int64,queue_size=1)
        self.drive_pub = rospy.Publisher("high_level/ackermann_cmd_mux/input/nav_0", AckermannDriveStamped, queue_size=1)
        self.driveInfo=AckermannDriveStamped()
        rospy.Timer(rospy.Duration(1.0 / 5), self.timer_callback) #0.2초마다 콜백

        self.laneWaypoint_x = 0.0
        self.laneWaypoint_y = 0.0
        self.coneWaypoint_x = 0.0
        self.coneWaypoint_y = 0.0
        self.avoid_angle = 0.0
    
    def missionNum(self, msg):   #미션 번호 받기
        self.mission=msg.state   #미션 번호
        self.static_obstacle_offset=msg.static_obstacle

    def lane_callback(self, msg):   #차선 주행 콜백
        self.laneWaypoint_x = msg.lane_x #웨이포인트 x좌표
        self.laneWaypoint_y = msg.lane_y #웨이포인트 y좌표

    def avoid_angle_callback(self, msg):
        self.avoid_angle = msg.angle

    def publish_data(self): #전체 퍼블리싱
        str = f"speed : {self.driveInfo.drive.speed}\tangle : {self.driveInfo.drive.steering_angle}"
        rospy.loginfo(str)
        self.driveInfo.drive.steering_angle -= 0.033  #조향각 오프셋
        self.drive_pub.publish(self.driveInfo)  #주행 정보 퍼블리싱
        self.lane_pub.publish(self.lane)        #차로 정보 퍼블리싱
        
    def timer_callback(self, _event):
        # if self.coneWaypoint_x == 0 and self.coneWaypoint_y == 0:# and self.laneWaypoint_x == 0 and self.laneWaypoint_y == 0:
        #     self.stop() #아무것도 검출이 되지 않으면 정지
        #     return

        self.follow_lane()  #그렇지 않으면 주행

    def calc_angle_speed(self):
        angle_parameter = 3

        if self.mission == StateNum.RUBBERCON_DRIVING:  #라바콘
            self.lane = -1
            angle = -1 * self.avoid_angle
        else:
            '''좌표 기준점 평행이동'''
            x = self.laneWaypoint_x - 160
            y = 240 - self.laneWaypoint_y
    
            try:
                angle = -1 * math.atan(x / y) / math.pi * 2 * 0.34 * angle_parameter #웨이포인트 각도를 -1~1로 변환 후 맵핑
            except:
                angle = 0.0

        '''테스트용 각도'''
        # angle = 0.0
        # (2.5 - abs(angle) / 0.34 / angle_parameter * 2.5) #조향각 연동 속도
        usualSpeed = 2.5 * 0.5  #일반 속도
        limitedSpeed = 2.5 * 0.14   #제한 속도

        if self.mission == StateNum.NORMAL_DRIVING_WITH_YOLO:
            speed = usualSpeed * 0.5
            angle = angle * 0.6

        elif self.mission == StateNum.NORMAL_DRIVING:    #평시주행
            speed = usualSpeed * 0.8
            angle = angle * 0.6

        elif self.mission == StateNum.SCHOOL_ZONE_SIGN_RECOGNITION:  #어린이보호구역
            speed = limitedSpeed
        elif self.mission == StateNum.SCHOOL_ZONE_CROSSING_RECOGNITION:
            speed = 0.0
        elif self.mission == StateNum.SCHOOL_ZONE_RESTART:
            speed = limitedSpeed

        elif self.mission == StateNum.DYNAMIC_OBSTACLE:  #동적장애물
            speed = 0.0
        
        elif self.mission == StateNum.RUBBERCON_DRIVING:  #라바콘 회피주행
            speed = usualSpeed * 0.475
            angle = angle * 0.025

        elif self.mission == StateNum.STATIC_OBSTACLE:  #정적장애물
            self.doSnake()
            speed = usualSpeed

        return angle, speed
    
    def likeSnake(self):
        self.driveInfo.header.stamp = rospy.Time.now()
        self.driveInfo.header.frame_id = "base_link"

        angle=self.lane*0.50
        speed = 2.5 * 0.25
        
        self.driveInfo.drive.steering_angle = angle
        self.driveInfo.drive.speed = speed
        self.publish_data()

    def doSnake(self):
        rate=rospy.Rate(100)    #100Hz로 설정


        for i in range(50): #0.40초 동안 조향
            self.likeSnake()
            rate.sleep()

        for i in range(90 if self.lane==-1 else 50): #0.80초 동안 직진
            self.driveInfo.drive.steering_angle = 0.0
            self.driveInfo.drive.speed = 2.5 * 0.2
            self.publish_data()
            rate.sleep()

        self.lane = -self.lane  #차로 상태 변경

        for i in range(25 if self.lane==-1 else 40): #0.32초 동안 반대 조향
            self.likeSnake()
            rate.sleep()

        self.mission = StateNum.NORMAL_DRIVING_WITH_YOLO   #미션 상태 평상시로 복귀

    def follow_lane(self):
        self.driveInfo.header.stamp = rospy.Time.now()
        self.driveInfo.header.frame_id = "base_link"
        
        angle, speed = self.calc_angle_speed()
        self.driveInfo.drive.steering_angle = angle
        self.driveInfo.drive.speed = speed

        self.publish_data()

    def stop(self):
        self.driveInfo.header.stamp = rospy.Time.now()
        self.driveInfo.header.frame_id = "base_link"
        self.driveInfo.drive.steering_angle = 0.0
        self.driveInfo.drive.speed = 0.0
        self.publish_data()
        rospy.loginfo("Stop Vehicle")

def run():
    rospy.init_node("control_example")
    Controller()
    rospy.spin()

if __name__ == "__main__":
    run()