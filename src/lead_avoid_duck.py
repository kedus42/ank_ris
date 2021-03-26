#!/usr/bin/env python3
import rospy
import time
from sensor_msgs.msg import CompressedImage, Joy, Image
from cv_bridge import CvBridge
from std_msgs.msg import String
import numpy as np
import cv2

rospy.init_node("lead_avoid")
avoid_pub=rospy.Publisher('/ank/joy', Joy, queue_size=30)
image_pub=rospy.Publisher('/ank/ducks', Image, queue_size=30)
bridge=CvBridge()

interval=1
steer_at=.5
speed=.6
backup_speed=-.4
avoidance_trigger=20
last_update=time.time()
lost=False
camwidth=640
camheight=480
previous_seq=0
skip_nimages=1

hmin, smin, vmin = 17, 184, 0
hmax, smax, vmax = 32, 255, 145
lower = np.array([hmin, smin, vmin])
upper = np.array([hmax, smax, vmax])

command=Joy()
i=0
while i < 8:
    command.axes.append(0)
    i+=1

indices=np.indices((camheight, camwidth))

def callback(image):
    global command, last_update, previous_seq
    if image.header.seq-previous_seq > skip_nimages: 
        previous_seq = image.header.seq
        arr=np.fromstring(image.data, np.uint8)
        img=cv2.imdecode(arr, cv2.IMREAD_COLOR)
        Hsvimg=cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask=cv2.inRange(Hsvimg, lower, upper)
        colored=cv2.bitwise_and(img, img, mask=mask)
        masked_img=bridge.cv2_to_imgmsg(mask)
        image_pub.publish(masked_img)
        mask_sum=np.sum(mask)
        if mask_sum>mask.shape[0]*mask.shape[1]*255/avoidance_trigger:
            num_yellow=mask_sum/255
            avgx=np.sum(mask*indices[1])
            avgx/=(255*num_yellow)
            if avgx<camwidth/2:
                command.axes[3]=steer_at
            else:
                command.axes[3]=-1*steer_at
        elif time.time()-last_update>interval and lost==False:
            command.axes[3]=np.random.uniform(-.5, .5)
            last_update=time.time()
        if lost==True:
            command.axes[1]=backup_speed
        else:
            command.axes[1]=speed
        avoid_pub.publish(command)
    else:
        pass

def lost_found_callback(msg):
    global lost
    if msg=="lost":
        lost=True
    else:
        lost=False

#follow_sub=rospy.Subscriber('/lead/lost', String, callback=lost_found_callback)
camera_sub=rospy.Subscriber('/ank/camera_node/image/compressed', CompressedImage, callback=callback)

rospy.spin()