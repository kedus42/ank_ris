#!/usr/bin/env python3
import rospy
import time
from sensor_msgs.msg import CompressedImage, Joy, Image
from cv_bridge import CvBridge
from std_msgs.msg import String
import numpy as np
import cv2

rospy.init("avoid")
avoid_pub=rospy.Publisher('/lead/joy', Joy, queue_size=30)
image_pub=rospy.Publisher('/lead/ducks', Image, queue_size=30)
bridge=CvBridge()

interval=5
steer_at=.5
speed=.6
backup_speed=-.4
lost=False

hmin, smin, vmin = 0, 0, 0
hmax, smax, vmax = 0, 0, 0

command=Joy()
i=0
while i < 8:
    command.axes.append(0)

def callback(image):
    global command, hmin, smin, vmin, hmax, smax, vmax, lost
    arr=np.fromstring(image.data, np.uint8)
    img=cv2.imdecode(arr, cv2.IMREAD_COLOR)
    Hsvimg=cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower = np.array([hmin, smin, vmin])
    upper = np.array([hmax, smax, vmax])
    mask=cv2.inRange(Hsvimg, lower, upper)
    colored=cv2.bitwise_and(img, img, mask=mask)
    colored_img=bridge.cv2_to_imgmsg(colored)
    image_pub.publish(colored_img)
    #find center of mass yellow in hsvimg
    if #mass yellow found:
        if #mass yellow in hsvimg centered on the left:
            #command.axes[3]=steer_at
        elif #mass yellow in hsvimg centered in the right:
            #commadn.axesp[3]=-1*steer_at
    elif #last_update-time.time()>interval and lost==False:
        #command.axes[3]=np.random.uniform(-.5, .5)
        #last_update=time.time()
    if lost==True:
        command.axes[1]=-1*speed
    else:
        command.axes[1]=backup_speed
    avoid_pub.publish(command)

def lost_found_callback(msg):
    global lost
    if msg=="lost":
        lost=True
    else:
        lost=False

follow_sub=rospy.Subscriber('/lead/lost', String, callback=lost_found_callback)
camera_sub=rospy.Subscriber('/lead/camera_node/image/compressed', CompressedImage, callback=callback)

rospy.spin()