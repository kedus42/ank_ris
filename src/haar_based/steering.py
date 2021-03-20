#!/usr/bin/env python3
import rospy
import time
from sensor_msgs.msg import CompressedImage, Joy, Image
from cv_bridge import CvBridge
import numpy as np
import cv2

nduckie=cv2.CascadeClassifier("../../haarcascades/haarcascade_frontalface_default.xml")

rospy.init_node('steering')
steering_pub=rospy.Publisher('/ank/joy', Joy, queue_size=30)
image_pub=rospy.Publisher('/ank/detections', Image, queue_size=30)
camwidth=640
camheight=480
move_threshhold=int(camwidth*.8)
action_threshhold=0
steer_at=.5
bridge=CvBridge()

def callback(image):
    arr=np.fromstring(image.data, np.uint8)
    img=cv2.imdecode(arr, cv2.IMREAD_COLOR)#CV_LOAD_IMAGE_COLOR
    command=Joy()
    duckies=nduckie.detectMultiScale(img, 1.1, 4)
    lowesty=camheight+1
    lowestx=camwidth+1
    highestx, highesty, lowestw, lowesth=0, 0, 0, 0
    highestw=camwidth
    highesth=camheight
    count=0
    action_threshhold=0
    i=0
    while i<8:
        command.axes.append(0)
        i+=1
    for x,y,w,h in duckies:
        cv2.rectangle(img, (x,y), (x+w, y+h), (0, 0, 255), 1)
        if x < lowestx:
            lowestx=x
            lowestw=w
        if x+w > highestx:
            highestx=x+w
            highestw=w
        if y < lowesty:
            lowesty=y
            lowesth=h
        if y+h > highesty:
            highesty=y+h
            highesth=h
        count+=1
    recx=lowestx
    recy=lowesty
    recw=highestx-lowestx
    rech=highesty-lowesty
    if count>action_threshhold:
        cv2.rectangle(img, (lowestx, lowesty), (lowestx+recw, lowesty+rech), (0, 255, 0), 1)
        if recx+recw/2 < int((camwidth/2)-camwidth/10):
            command.axes[3]=steer_at
        elif recx+recw/2 > int((camwidth/2)+camwidth/10):
            command.axes[3]=-1*steer_at
        if recw < move_threshhold:
            command.axes[1]=1-recw/camwidth
        elif recw > move_threshhold:
            command.axes[1]=.2
    else:
        command.axes[1]=0
        command.axes[3]=0
    detect_msg=bridge.cv2_to_imgmsg(img, 'bgr8')
    image_pub.publish(detect_msg)
    steering_pub.publish(command)

camera_sub=rospy.Subscriber('/ank/camera_node/image/compressed', CompressedImage, callback=callback)
rospy.spin()