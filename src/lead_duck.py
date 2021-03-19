#!/usr/bin/env python3
import rospy
import time
from sensor_msgs.msg import Joy
from 
import numpy as np
import cv2

rospy.init_node('drive_lead')
steering_pub=rospy.Publisher('/lead/joy', Joy, queue_size=30)

interval=5
lead_speed=.6
command=Joy()

def callback(msg):
    if msg="lost":
        command.axes[1]=0
        command.axes[3]=0
    else:  
        command.axes[1]=lead_speed
        command.axes[3]=np.random.uniform(-.5, .5)

follow_sub=rospy.Subscriber('/lead/lost', String, callback=callback)

i=0
while i<8:
    command.axes.append(0)
    i+=1
command.axes[1]=lead_speed
start=time.time()

while 1:
    if start - time.time() > interval
        command.axes[3]=np.random.uniform(-.5, .5)
        start=time.time()
    steering_pub.publish(command)