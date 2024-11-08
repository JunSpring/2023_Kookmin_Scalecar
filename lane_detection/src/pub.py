#!/usr/bin/env python
import os
import rospy
import cv2
import numpy as np
from std_msgs.msg import Int64
from sensor_msgs.msg import CompressedImage
from lane_detection.msg import lane_msg
from cv_bridge import CvBridge

Width = 640
Height = 480

warp_img_w = 320
warp_img_h = 240

warpx_margin = 160
warpy_margin = 50
temp_margin = 300

nwindows = 20
margin = 20
minpix = 5

lane_bin_th = 145

count = 0

warp_src = np.array([
    [Width/2 - warpx_margin, Height/2 + warpy_margin],  
    [-temp_margin, Height],
    [Width/2 + warpx_margin, Height/2 + warpy_margin],
    [Width + temp_margin, Height]
], dtype=np.float32)

warp_dist = np.array([
    [0, 0],
    [0, warp_img_h],
    [warp_img_w, 0],
    [warp_img_w, warp_img_h]
], dtype=np.float32)

class CameraReceiver():
    state = -1
    def __init__(self):
        rospy.loginfo("Camera Receiver Object is Created")
        rospy.Subscriber("/usb_cam/image_rect_color/compressed", CompressedImage, self.Callback)
        rospy.Subscriber("/whatLane", Int64, self.state_Callback)
        self.pub = rospy.Publisher("/lane_pub", lane_msg, queue_size = 10)

    def state_Callback(self, msg):
        self.state = -msg.data
        print("2차로" if (self.state == -1) else "1차로")

    def Callback(self, data):
        global Width, Height, cap
        global count

        bridge = CvBridge()

        count += 1

        if(count % 2 == 0):
            image = bridge.compressed_imgmsg_to_cv2(data,"bgr8")
            state = self.state
            warp_img, M, Minv = warp_image(image, warp_src, warp_dist, (warp_img_w, warp_img_h))
            left_fit, right_fit, avex, avey, leftx_current, lefty, rightx_current, righty = warp_process_image(state, warp_img)
            lane_img = draw_lane(image, warp_img, Minv, left_fit, right_fit, avex, avey, leftx_current, lefty, rightx_current, righty)
            msg = lane_msg()
            msg.lane_x = avex
            msg.lane_y = avey
            self.pub.publish(msg)

            cv2.imshow("warp", warp_img)
            cv2.imshow("lane", lane_img)

            cv2.waitKey(1)

def warp_image(img, src, dst, size):
    M = cv2.getPerspectiveTransform(src, dst)
    Minv = cv2.getPerspectiveTransform(dst, src)
    warp_img = cv2.warpPerspective(img, M, size, flags=cv2.INTER_LINEAR)

    return warp_img, M, Minv

def warp_process_image(state, img):
    global lefty
    global righty
    global nwindows 
    global margin 
    global minpix 
    global lane_bin_th 

    blur = cv2.GaussianBlur(img,(5, 5), 0) 
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

    lower_white = np.array([0, 0, 180])
    upper_white = np.array([180, 255, 255])

    lane = cv2.inRange(hsv, lower_white, upper_white)

    cv2.imshow("w", lane)
   
    cv2.waitKey(1)

    histogram = np.sum(lane[lane.shape[0]//2:,:], axis=0) 
    midpoint = np.int(histogram.shape[0]/2) 
    leftx_current = np.argmax(histogram[:midpoint]) 
    rightx_current = np.argmax(histogram[midpoint:]) + midpoint 

    window_height = np.int(lane.shape[0]/nwindows) 
    nz = lane.nonzero() 

    left_lane_inds = [] 
    right_lane_inds = []
    
    lx, ly, rx, ry = [], [], [], [] 

    out_img = np.dstack((lane, lane, lane))*255

    for window in range(nwindows): 

        win_yl = lane.shape[0] - (window+1) * window_height 
        win_yh = lane.shape[0] - window * window_height 

        win_xll = leftx_current - margin 
        win_xlh = leftx_current + margin 
        win_xrl = rightx_current - margin 
        win_xrh = rightx_current + margin

        cv2.rectangle(out_img, (win_xll, win_yl), (win_xlh,win_yh), (0,255,0), 2) 
        cv2.rectangle(out_img, (win_xrl, win_yl), (win_xrh,win_yh), (0,255,0), 2) 
     
        good_left_inds = ((nz[0] >= win_yl)&(nz[0] < win_yh)&(nz[1] >= win_xll)&(nz[1] < win_xlh)).nonzero()[0] 
        good_right_inds = ((nz[0] >= win_yl)&(nz[0] < win_yh)&(nz[1] >= win_xrl)&(nz[1] < win_xrh)).nonzero()[0] 

        left_lane_inds.append(good_left_inds) 
        right_lane_inds.append(good_right_inds)
        #lefty = 0
        #righty = 0

        if len(good_left_inds) > minpix and state == 1:# 왼쪽 주행 중
            leftx_current = np.int(np.mean(nz[1][good_left_inds])) 
            lefty = np.int(np.mean(nz[0][good_left_inds])) 
            rightx_current = leftx_current + 125
            righty = lefty

        if len(good_right_inds) > minpix and state == -1: # 오른쪽 주행 중 
            rightx_current = np.int(np.mean(nz[1][good_right_inds])) 
            righty = np.int(np.mean(nz[0][good_right_inds]))
            leftx_current = rightx_current - 180
            lefty = righty


        lx.append(leftx_current) 
        ly.append((win_yl + win_yh)/2)

        rx.append(rightx_current)
        ry.append((win_yl + win_yh)/2)

    left_lane_inds = np.concatenate(left_lane_inds) 
    right_lane_inds = np.concatenate(right_lane_inds)
    
    lfit = np.polyfit(np.array(ly),np.array(lx),2) 
    rfit = np.polyfit(np.array(ry),np.array(rx),2)
    
    out_img[nz[0][left_lane_inds], nz[1][left_lane_inds]] = [255, 0, 0] 
    out_img[nz[0][right_lane_inds] , nz[1][right_lane_inds]] = [0, 0, 255]

    #if leftx_current == 0 and lefty == 5:
    #    leftx_current = rightx_current - 156
    #    lefty = righty

    #elif rightx_current == 0 and righty == 5:
    #    rightx_current = leftx_current + 156
    #    righty = lefty

    # if state == 0:# 왼쪽 주행 중
    #     rightx_current = leftx_current + 80
    
    # elif state == 1: # 오른쪽 주행 중
    #     leftx_current = rightx_current - 80

    avex = (leftx_current + rightx_current)//2
    avey = (lefty + righty)//2

    cv2.line(out_img,(leftx_current, lefty), (rightx_current, righty), (255, 0, 255), 2)
    cv2.circle(out_img,(avex, avey), 5, (0, 255, 255), -1)
    cv2.circle(out_img, (leftx_current, lefty), 5, (0,0,255), -1)
    cv2.circle(out_img, (rightx_current, righty), 5, (255,0,0), -1)

    cv2.imshow("viewer", out_img)

    return lfit, rfit, avex, avey, leftx_current, lefty, rightx_current, righty

def draw_lane(image, warp_img, Minv, left_fit, right_fit, avex, avey, leftx_current, lefty, rightx_current, righty):
    global Width, Height
    yMax = warp_img.shape[0]
    ploty = np.linspace(0, yMax - 1, yMax)
    color_warp = np.zeros_like(warp_img).astype(np.uint8)
    
    left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
    right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]
    
    pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
    pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, ploty])))]) 
    pts = np.hstack((pts_left, pts_right))
    

    cv2.circle(color_warp,(avex, avey), 10, (0, 0, 255), -1)
    newwarp = cv2.warpPerspective(color_warp, Minv, (Width, Height))

    return cv2.addWeighted(image, 1, newwarp, 0.3, 0)

def run():
    rospy.init_node("ld_pub")
    #rate = rospy.Rate(1)
    cam = CameraReceiver()
    rospy.spin()

if __name__=="__main__":
    try:
        run()
    except rospy.ROSInterruptException:
        pass