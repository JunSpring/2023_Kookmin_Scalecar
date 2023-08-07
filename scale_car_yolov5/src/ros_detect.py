#!/usr/bin/env python3

import rospy
import cv2
#from sensor_msgs.msg import Image
#from sensor_msgs.msg import CompressedImage
#from cv_bridge import CvBridge
from std_msgs.msg import Int64
from scale_car_yolov5.msg import Yolo_Objects, Objects

import os
import sys
from pathlib import Path
import numpy as np

import torch

# yolov5 submodule을 path에 추가
FILE = Path(__file__).resolve()
ROOT = FILE.parents[0] / "yolov5"
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # ROOT를 PATH에 추가
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # 관련 path

from models.common import DetectMultiBackend
from utils.general import (cv2, non_max_suppression, scale_boxes)
from utils.plots import Annotator, colors
from utils.augmentations import letterbox

class YoloV5_ROS():
    rospy.init_node("scale_car_yolov5")
    pub = rospy.Publisher("yolov5_pub", data_class=Yolo_Objects, queue_size=10)

    source = rospy.get_param("~source")
    weights = rospy.get_param("~weights")  # model path
    data = rospy.get_param("~data")  # dataset.yaml path
    device = torch.device(rospy.get_param("~device"))  # cuda device, i.e. 0 or 0,1,2,3 or cpu

    #weights = "/home/wego/catkin_ws/src/scale_car_yolov5/src/yolov5/runs/train/scale_car_train_result3/weights/best.pt"
    #data = "/home/wego/catkin_ws/src/scale_car_yolov5/src/yolov5/data/scale_car.yaml"
    #device = torch.device("cpu")

    imgsz = (160, 160)
    
    conf_thres = 0.25  # confidence threshold
    iou_thres = 0.45  # NMS IOU threshold
    max_det = 10  # maximum detections per image
    classes = None  # filter by class
    agnostic_nms = False  # class-agnostic NMS
    line_thickness = 3  # bounding box thickness (pixels)

    hide_labels = False  # hide labels
    hide_conf = False  # hide confidences

    # 카메라 Callback
    cap = cv2.VideoCapture(source)
    
    # Load model
    model = DetectMultiBackend(weights, device=device, dnn=False, data=data, fp16=False)
    stride, names, pt = model.stride, model.names, model.pt
    count = 0

    while cap.isOpened() and not rospy.is_shutdown():
        c = -1 # class (-1: 물체 없음)
        count += 1
        ret, img = cap.read()
        cv2.namedWindow('result', cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)  # allow window resize (Linux)
        
        im0s = img
        #cv2.resizeWindow('result', im0s.shape[1], im0s.shape[0])

        # Run inference
        model.warmup(imgsz=(1 if pt or model.triton else bs, 3, *imgsz))  # warmup
        #seen = 0
        #s = ''
        #dt = (Profile(), Profile(), Profile())

        im = letterbox(img, imgsz, stride=stride, auto=pt)[0]  # padded resize
        im = im.transpose((2, 0, 1))[::-1]  # HWC to CHW, BGR to RGB
        im = np.ascontiguousarray(im)  # contiguous

        im = torch.from_numpy(im).to(model.device)
        im = im.half() if model.fp16 else im.float()  # uint8 to fp16/32
        im /= 255  # 0 - 255 to 0.0 - 1.0
        if len(im.shape) == 3:
            im = im[None]  # expand for batch dim

        # Inference
        pred = model(im, augment=False, visualize=False)

        # NMS
        pred = non_max_suppression(pred, conf_thres, iou_thres, classes, agnostic_nms, max_det=max_det)

        # Process predictions
        for i, det in enumerate(pred):  # per image
            #seen += 1
            #im0 = im0s.copy()

            #s += '%gx%g ' % im.shape[2:]  # print string
            gn = torch.tensor(im0s.shape)[[1, 0, 1, 0]]  # normalization gain whwh
            annotator = Annotator(im0s, line_width=line_thickness, example=str(names))

            if len(det):
                # Rescale boxes from img_size to im0 size
                det[:, :4] = scale_boxes(im.shape[2:], det[:, :4], im0s.shape).round()

                # Print results
                #for c in det[:, 5].unique():
                    #n = (det[:, 5] == c).sum()  # detections per class
                    #s += f"{n} {names[int(c)]}{'s' * (n > 1)}, "  # add to string

            # Write results
            for *xyxy, conf, cls in reversed(det):

                # Add bbox to image
                c = int(cls)  # integer class
                label = None if hide_labels else (names[c] if hide_conf else f'{names[c]} {conf:.2f}')
                annotator.box_label(xyxy, label, color=colors(c, True))

                # box coordinate
                x1 = xyxy[2].tolist()
                y1 = xyxy[3].tolist()
                x2 = xyxy[0].tolist()
                y2 = xyxy[1].tolist()

                # msg publish
                msg = Yolo_Objects()
                msg.yolo_objects.append(Objects(c, x1, x2, y1, y2))
                pub.publish(msg)
                
                #img = cv2.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), (255, 0, 0), 3)
        
        cv2.imshow('result', im0s)
        cv2.waitKey(1)  # 1 millisecond
            
def run():
    detect = YoloV5_ROS()
    rospy.spin()

if __name__ == '__main__':
    run()
