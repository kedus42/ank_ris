#!/usr/bin/env python2.7
import rospy
import time
from sensor_msgs.msg import CompressedImage, Joy
import numpy as np

nduckie=cv2.CascadeClassifier("../haarcascades/duckie_cascade_stage10.xml")

rospy.init_node('steering')
steering_pub=rospy.Publisher('/ank/joy', Joy, queue_size=30)
camwidth=50
camheight=50
move_threshhold=int(camwidth*.8)

def callback(image):
    arr=np.fromstring(image.data, np.uint8)
    img=cv2.imdecode(arr, cv2.IMREAD_COLOR)#CV_LOAD_IMAGE_COLOR
    command=Joy()
    duckie=nduckie.detectMultiScale(img, 1.1, 4)
    count=0
    while i<8:
        command.axes.append(0)
    for x,y,w,h in duckie:
        cv2.rectangle(img, (x,y), (x+w, y+h), (0, 255, 0), 1)
        cv2.imshow("found this duckie", img)
        if x+w/2 < int((camwidth/2)-camwidth/10):
            command.axes[3]=.5#wherever left is
        elif x+w/2 > int((camwidth/2)+camwidth/10):
            command.axes[3]=-.5#wherever right is
        if w < move_threshhold:
            command.axes[1]=w/(camwidth)
        elif w > move_threshhold:
            command.axes[1]=-.2
        else:
            command.axes[1]=0
        steering_pub.publish(command)
        count+=1
        if count =1:
            break

camera_sub=rospy.Subscriber('/ank/camera_node/image/compressed', CompressedImage, callback=callback)
rospy.spin()