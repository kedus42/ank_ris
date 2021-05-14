#!/usr/bin/env python3
import rospy, time, rospkg
from sensor_msgs.msg import CompressedImage, Joy, Image
from std_msgs.msg import String, Float32
from cv_bridge import CvBridge
import numpy as np
import cv2

rospack=rospkg.RosPack()
path=rospack.get_path('ank_ris')

#Loading lower body detecting haarcascade
nbody=cv2.CascadeClassifier(path+"/haarcascades/haarcascade_lowerbody.xml")

rospy.init_node('lead_lb')
name=rospy.get_param('duckie')
#Setting up publishers and subs
steering_pub=rospy.Publisher('/'+name+'/joy', Joy, queue_size=30)
image_pub=rospy.Publisher('/'+name+'/detections', Image, queue_size=30)
speed=rospy.get_param('speed')

camwidth=640
camheight=480

#Threshholds for activation of movement
move_threshhold=int(camwidth*.8)
h_threshold=int(camheight*0)
action_threshhold=0

steer_at=speed*.5
speed=speed
#steer_at=.2
#speed=.5
move=True
bridge=CvBridge()
switch="cam"

#Callback called every 0.5 seconds
def callback(timer_info):
    global switch
    switch=rospy.get_param("/switch")
    #Depending on rosparam "switch" image is processed for human detection or skipped
    if switch=="lb":
        image=rospy.wait_for_message('/'+name+'/camera_node/image/compressed', CompressedImage)
        arr=np.fromstring(image.data, np.uint8)
        img=cv2.imdecode(arr, cv2.IMREAD_COLOR)#CV_LOAD_IMAGE_COLOR
        command=Joy()

        #Find lower body using loaded haarcascade
        bodies=nbody.detectMultiScale(img, 1.2, 1)
        highestw=0
        count=0
        action_threshhold=0
        highesth=0
        i=0
        while i<8:
            command.axes.append(0)
            i+=1
        
        #Loop to run through all detected bodies and save the largest one
        for x,y,w,h in bodies:
            cv2.rectangle(img, (x,y), (x+w, y+h), (0, 0, 255), 1)
            if w > highestw:
                trackx=x
                tracky=y
                trackh=h
                highesth=h
                highestw=w
            count+=1

        #if bodies were detected, steer in the direction of the largest one
        if count > 0 and highesth>h_threshold and move:
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

        #Print images bakc in the form of feedback
        detect_msg=bridge.cv2_to_imgmsg(img, 'bgr8')
        image_pub.publish(detect_msg)
        #cv2.imshow("detected", img)
        steering_pub.publish(command)
    else:
        pass

#Deprecated callback
def follow_callback(msg):
    global move
    if msg=="lost":
        move=False
    if msg=="found":
        move=True

#Callback for when speed is changed via speed topic
def changeSpeed(new_speed):
    global speed, steer_at
    speed=new_speed.data
    steer_at=new_speed.data*.5

#Timer to call callback every half a second
timer=rospy.Timer(rospy.Duration(0.5), callback)
#camera_sub=rospy.Subscriber('/lead/camera_node/image/compressed', CompressedImage, callback=callback)
follow_sub=rospy.Subscriber('/lead/lost', String, callback=callback)

#Callback to change speed when a new one is received
speed_sub=rospy.Subscriber('/'+name+'/speed', Float32, callback=changeSpeed)
rospy.spin()