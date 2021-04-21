#!/usr/bin/env python3
import rospy, sys, rospkg, cv2
from PyQt5 import QtWidgets, QtGui, QtCore
from PyQt5.QtWidgets import QApplication, QMainWindow, QMessageBox, QInputDialog
from sensor_msgs.msg import Joy, CompressedImage
from duckietown_msgs.msg import Pose2DStamped
import numpy as np

rospy.init_node("nav_home")
rospack=rospkg.RosPack()
path=rospack.get_path('ank_ris')

home=rospy.wait_for_message('/lead/velocity_to_pose_node/pose', Pose2DStamped)
return_home=False
command=Joy()
i=0
while i<8:
    command.axes.append(0)
    i+=1

class window(QMainWindow):
    def __init__(self):
        super(window, self).__init__()
        self.setGeometry(400, 150, 900, 400)
        self.setStyleSheet("background : grey")

        self.b1 = QtWidgets.QPushButton(self)
        self.b1.setGeometry(550, 150, 300, 50)
        self.b1.setText("Return home")
        self.b1.clicked.connect(self.b1_clicked)

        self.b2 = QtWidgets.QPushButton(self)
        self.b2.setGeometry(550, 210, 300, 50)
        self.b2.setText("Start rand walk")
        self.b2.clicked.connect(self.b2_clicked)

        self.b3 = QtWidgets.QPushButton(self)
        self.b3.setGeometry(550, 270, 300, 50)
        self.b3.setText("Stop")
        self.b3.clicked.connect(self.b2_clicked)

        self.b4 = QtWidgets.QPushButton(self)
        self.b4.setGeometry(550, 330, 300, 50)
        self.b4.setText("Set home as current pose")
        self.b4.clicked.connect(self.b2_clicked)

        self.l1 = QtWidgets.QLabel(self)
        self.l1.setGeometry(550, 50, 300, 20)
        self.l1.setText("Home: "+str(round(home.x, 2))+"x  "+str(round(home.y, 2))+ "y  "+str(round(home.theta, 2))+" theta")

        self.l2 = QtWidgets.QLabel(self)
        self.l2.setGeometry(550, 100, 300, 20)
        self.l2.setText("Current position: ")

        self.l3 = QtWidgets.QLabel(self)
        self.l3.setGeometry(10, 10, 512, 384)
        self.l3.setPixmap(QtGui.QPixmap(path+'/imgs/cam_feed.jpg'))
        self.l3.setScaledContents(1)


    def b1_clicked(self):
        pass
    def b2_clicked(self):
        pass

app=QApplication(sys.argv)
win=window()
win.show()

def odom_callback(pose):
    global return_home
    if return_home == False:
        win.l2.setText("Current position: "+str(round(pose.x, 2))+"x  "+str(round(pose.y, 2))+ "y  "+str(round(pose.theta, 2))+" theta")
    else:
        if np.linalg.norm(np.array([home.x, home.y, home.z])-np.array([home.x, home.y, home.z])) < 5:
            return_home=False

def cam_feed(timer_info):
    global win
    image=rospy.wait_for_message('/lead/camera_node/image/compressed', CompressedImage)
    arr=np.fromstring(image.data, np.uint8)
    img=cv2.imdecode(arr, cv2.IMREAD_COLOR)#CV_LOAD_IMAGE_COLOR
    cv2.imwrite(path+'/imgs/cam_feed.jpg', img)
    win.l3.setPixmap(QtGui.QPixmap(path+'/imgs/cam_feed.jpg'))
    win.l3.setScaledContents(1)
    
rospy.Subscriber('/lead/velocity_to_pose_node/pose', Pose2DStamped, odom_callback)
rospy.Timer(rospy.Duration(.5), cam_feed)
sys.exit(app.exec_())