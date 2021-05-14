#!/usr/bin/env python3
import rospy
import time
from sensor_msgs.msg import CompressedImage, Joy, Image
from cv_bridge import CvBridge
from std_msgs.msg import String, Float32
import numpy as np
import cv2

rospy.init_node('steering')
name=rospy.get_param('duckie')

#Declaring publishers
steering_pub=rospy.Publisher('/'+name+'/joy', Joy, queue_size=30)
image_pub=rospy.Publisher('/'+name+'/detections', Image, queue_size=30)
speed=rospy.get_param('/speed')
lead_pub=rospy.Publisher('/lead/lost', String, queue_size=30)
camwidth=640
camheight=480

#Threshholds for movement activation
move_threshhold=int(camwidth*.25)
action_threshhold=2
steer_at=speed*.5
bridge=CvBridge()
idle_time_steps=0
tolerable_idlness=2
fps=2.0
switch="cam"


#Function to stop duck
def stopFollower():
    command=Joy()
    i=0
    while i<8:
        command.axes.append(0)
        i+=1
    command.axes[1]=0
    command.axes[3]=0
    steering_pub.publish(command)

rospy.on_shutdown(stopFollower)

#Callback called every 0.5 seconds
def callback(t_info):
    global idle_time_steps, switch
    switch=rospy.get_param("/switch")
    #Depending on rosparam "switch" image is processed for human detection or skipped
    if switch=="sc":
        image=rospy.wait_for_message('/'+str(name)+'/camera_node/image/compressed', CompressedImage)
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

        #Retrieve all circles detected in image that fit the given parameters
        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, 1, param1=80, param2=25, minRadius=0, maxRadius=20)

        #If there are circles meeting these parameters, find their average
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

        #Draw the average circle as a green circle if it is above the min threshhold
        if avgr>move_threshhold:
            cv2.circle(img, (int(avgx), int(avgy)), int(avgr), (0, 255, 0), 1)
            command.axes[1]=0
            command.axes[3]=0

        #If there are enough circles to act on, steer in the direction of 
        elif count>action_threshhold:
            cv2.circle(img, (int(avgx), int(avgy)), int(avgr), (0, 255, 0), 1)
            if avgx < int((camwidth/2)-camwidth/10):
                command.axes[3]=steer_at#*(camwidth/2-avgx)
            elif avgx > int((camwidth/2)+camwidth/10):
                command.axes[3]=-1*steer_at#*(avgx-camwidth/2)/(camwidth/2)
            command.axes[1]=speed
            lead_pub.publish("found")
            prev_move=time.time()
            idle_time_steps=0
        elif idle_time_steps >= tolerable_idlness:
            command.axes[1]=0
            command.axes[3]=0
        else:
            command.axes[1]=0
            command.axes[3]=0
            idle_time_steps+=1
            lead_pub.publish("lost")

        #Print feedback images
        detect_msg=bridge.cv2_to_imgmsg(img, 'bgr8')
        image_pub.publish(detect_msg)
        steering_pub.publish(command)
    else:
        pass

#Callback for when a new speed is published
def changeSpeed(new_speed):
    global speed, steer_at
    speed=new_speed.data
    steer_at=new_speed.data*.5

#Timer to call the callbakc every .5 seconds
timer=rospy.Timer(rospy.Duration(1.0/fps), callback)
speed_sub=rospy.Subscriber('/'+name+'/speed', Float32, callback=changeSpeed)
rospy.spin()