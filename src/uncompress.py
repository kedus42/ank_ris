#!/usr/bin/env python3
import rospy
import time
from sensor_msgs.msg import CompressedImage, Joy, Image
from cv_bridge import CvBridge
from std_msgs.msg import String
import numpy as np
import cv2

rospy.init_node('uncompress')

pub=rospy.Publisher('/ank/image', Image, queue_size=30)
bridge=CvBridge()

def callback(image):
    arr=np.fromstring(image.data, np.uint8)
    img=cv2.imdecode(arr, cv2.IMREAD_COLOR)#CV_LOAD_IMAGE_COLOR
    img=bridge.cv2_to_imgmsg(img)
    pub.publish(img)
    
sub=rospy.Subscriber('/ank/camera_node/image/compressed', CompressedImage, callback=callback)
rospy.spin()