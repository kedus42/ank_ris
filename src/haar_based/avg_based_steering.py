#!/usr/bin/env python3
import rospy
import time
from sensor_msgs.msg import CompressedImage, Joy, Image
from cv_bridge import CvBridge
import numpy as np
import cv2

#nduckie=cv2.CascadeClassifier("../../haarcascades/duckie_cascade_stage12.xml")
nduckie=cv2.CascadeClassifier("../../haarcascades/haarcascade_lowerbody.xml")


rospy.init_node('steering')
steering_pub=rospy.Publisher('/ank/joy', Joy, queue_size=30)
image_pub=rospy.Publisher('/ank/detections', Image, queue_size=30)
camwidth=640
camheight=480
move_threshhold=int(camwidth*.6)
action_threshhold=5
bridge=CvBridge()

def callback(image):
    arr=np.fromstring(image.data, np.uint8)
    img=cv2.imdecode(arr, cv2.IMREAD_COLOR)#CV_LOAD_IMAGE_COLOR
    command=Joy()
    duckies=nduckie.detectMultiScale(img, 1.1, 4)
    count=0
    action_threshhold=5
    avgx, avgy, avgw, avgh, i=0, 0, 0, 0, 0
    while i<8:
        command.axes.append(0)
        i+=1
    for x,y,w,h in duckies:
        cv2.rectangle(img, (x,y), (x+w, y+h), (0, 0, 255), 1)
        avgx+=x
        avgw+=w
        avgy+=y
        avgh+=h
        count+=1
    if count>action_threshhold:
        avgx/=count
        avgw/=count
        avgy/=count
        avgh/=count
        cv2.rectangle(img, (avgx, avgy), (avgx+avgw, avgy+avgh), (0, 255, 0), 1)
        if avgx+avgw/2 < int((camwidth/2)-camwidth/10):
            command.axes[3]=.5
        elif avgx+avgw/2 > int((camwidth/2)+camwidth/10):
            command.axes[3]=-.5
        if avgw < move_threshhold:
            command.axes[1]=1-avgw/camwidth
        elif avgw > move_threshhold:
            command.axes[1]=.2
    else:
        command.axes[1]=0
        command.axes[3]=0
    detect_msg=bridge.cv2_to_imgmsg(img, 'bgr8')
    image_pub.publish(detect_msg)
    #steering_pub.publish(command)

camera_sub=rospy.Subscriber('/ank/camera_node/image/compressed', CompressedImage, callback=callback)
rospy.spin()