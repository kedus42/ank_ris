#!/usr/bin/env python3
import rospy
import time
from sensor_msgs.msg import CompressedImage, Joy, Image
from cv_bridge import CvBridge
from std_msgs.msg import String
import numpy as np
import cv2

rospy.init_node('steering')
steering_pub=rospy.Publisher('/ank/joy', Joy, queue_size=30)
image_pub=rospy.Publisher('/ank/detections', Image, queue_size=30)
lead_pub=rospy.Publisher('/lead/lost', String, queue_size=30)
camwidth=640
camheight=480
move_threshhold=int(camwidth*.8)
action_threshhold=5
steer_at=.5
speed=.6
previous_seq=0
skip_nimages = 1
bridge=CvBridge()

def callback(image):
    global previous_seq
    if image.header.seq - previous_seq > skip_nimages:
        previous_seq = image.header.seq
        arr=np.fromstring(image.data, np.uint8)
        img=cv2.imdecode(arr, cv2.IMREAD_COLOR)#CV_LOAD_IMAGE_COLOR
        command=Joy()
        avgx=0
        avgy=0
        avgr=0
        i=0
        count=0
        while i<8:
            command.axes.append(0)
            i+=1
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, 1, param1=80, param2=25, minRadius=0, maxRadius=20)
        if circles is not None:
            circles = np.uint16(np.around(circles))
            for dim in circles[0,:]: 
                cv2.circle(img, (dim[0], dim[1]), dim[2], (0, 0, 255), 1)
                avgx+=dim[0]
                avgy+=dim[1]
                avgr+=dim[2]
                count+=1
            avgx/=count
            avgy/=count
            avgr/=count
        if count>action_threshhold:
            cv2.circle(img, (int(avgx), int(avgy)), int(avgr), (0, 255, 0), 1)
            if avgx < int((camwidth/2)-camwidth/10):
                command.axes[3]=steer_at#*(camwidth/2-avgx)
            elif avgx > int((camwidth/2)+camwidth/10):
                command.axes[3]=-1*steer_at#*(avgx-camwidth/2)/(camwidth/2)
            command.axes[1]=speed
            lead_pub.publish("found")
        else:
            command.axes[1]=0
            command.axes[3]=0
            lead_pub.publish("lost")
        detect_msg=bridge.cv2_to_imgmsg(img, 'bgr8')
        image_pub.publish(detect_msg)
        steering_pub.publish(command)
    else:
        pass

camera_sub=rospy.Subscriber('/ank/camera_node/image/compressed', CompressedImage, callback=callback)
rospy.spin()