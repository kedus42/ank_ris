#!/usr/bin/env python3
import rospy
import numpy as np
import cv2

def empty():
    pass

cv2.namedWindow("Trackbars")
cv2.resizeWindow("Trackbars", 600, 350)
cv2.createTrackbar("Hue Min", "Trackbars", 17, 179, empty)
cv2.createTrackbar("Hue Max", "Trackbars", 88, 179, empty)
cv2.createTrackbar("Sat Min", "Trackbars", 31, 255, empty)
cv2.createTrackbar("Sat Max", "Trackbars", 255, 255, empty)
cv2.createTrackbar("val Min", "Trackbars", 0, 255, empty)
cv2.createTrackbar("Val Max", "Trackbars", 255, 255, empty)

def callback(image):
    arr=np.fromstring(image.data, np.uint8)
    img=cv2.imdecode(arr, cv2.IMREAD_COLOR)
    Hsvimg=cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    cv2.imshow("a",Hsvimg)
    hmin=cv2.getTrackbarPos("Hue Min", "Trackbars")
    hmax=cv2.getTrackbarPos("Hue Max", "Trackbars")
    smin=cv2.getTrackbarPos("Sat Min", "Trackbars")
    smax=cv2.getTrackbarPos("Sat Max", "Trackbars")
    vmin=cv2.getTrackbarPos("Val Min", "Trackbars")
    vmax=cv2.getTrackbarPos("Val Max", "Trackbars")
    lower = np.array([hmin, smin, vmin])
    upper = np.array([hmax, smax, vmax])
    mask=cv2.inRange(Hsvimg, lower, upper)
    colored=cv2.bitwise_and(img, img, mask=mask)
    cv2.imshow("colored", colored)
    cv2.imshow("Masked", mask)

camera_sub=rospy.Subscriber('/lead/camera_node/image/compressed', CompressedImage, callback=callback)