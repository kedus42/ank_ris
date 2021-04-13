#!/usr/bin/env python3
import rospy
import time
from sensor_msgs.msg import CompressedImage, Joy, Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import numpy as np
import cv2

nbody=cv2.CascadeClassifier("../../haarcascades/haarcascade_lowerbody.xml")

rospy.init_node('lead_lb')
steering_pub=rospy.Publisher('/lead/joy', Joy, queue_size=30)
image_pub=rospy.Publisher('/lead/detections', Image, queue_size=30)
camwidth=640
camheight=480
move_threshhold=int(camwidth*.8)
action_threshhold=0
steer_at=.05
speed=.2
move=True
bridge=CvBridge()

def callback(image):
    rospy.wait_for_message('/lead/camera_node/image/compressed', CompressedImage)
    arr=np.fromstring(image.data, np.uint8)
    img=cv2.imdecode(arr, cv2.IMREAD_COLOR)#CV_LOAD_IMAGE_COLOR
    command=Joy()
    bodies=nbody.detectMultiScale(img, 1.2, 1)
    highestw=0
    count=0
    action_threshhold=0
    i=0
    while i<8:
        command.axes.append(0)
        i+=1
    for x,y,w,h in bodies:
        cv2.rectangle(img, (x,y), (x+w, y+h), (0, 0, 255), 1)
        if w > highestw:
            trackx=x
            tracky=y
            trackh=h
            hgihestw=w
        count+=1
    if count > 0 and move:
        cv2.rectangle(img, (trackx, tracky), (trackx+highestw, tracky+trackh), (0, 255, 0), 1)
        if trackx+highestw/2 < int((camwidth/2)-camwidth/10):
            command.axes[3]=steer_at#*((trackx+highestw/2)-camwidth/2)/(camwidth/2)
        elif trackx+highestw/2 > int((camwidth/2)+camwidth/10):
            command.axes[3]=-1*steer_at#*((trackx+highestw/2)-camwidth/2)/(camwidth/2)
        if highestw < move_threshhold:
            command.axes[1]=speed
        elif highestw > move_threshhold:
            command.axes[1]=-1*speed
    else:
        command.axes[1]=0
        command.axes[3]=0
    detect_msg=bridge.cv2_to_imgmsg(img, 'bgr8')
    image_pub.publish(detect_msg)
    #cv2.imshow("detected", img)
    steering_pub.publish(command)

def follow_callback(msg):
    global move
    if msg=="lost":
        move=False
    if msg=="found":
        move=True

timer=rospy.Timer(rospy.Duration(0.5), callback)
#camera_sub=rospy.Subscriber('/lead/camera_node/image/compressed', CompressedImage, callback=callback)
follow_sub=rospy.Subscriber('/lead/lost', String, callback=callback)
rospy.spin()