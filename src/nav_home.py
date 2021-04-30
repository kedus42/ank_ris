#!/usr/bin/env python3
import rospy, sys, rospkg, cv2, math
from PyQt5 import QtWidgets, QtGui, QtCore
from PyQt5.QtWidgets import QApplication, QMainWindow, QMessageBox, QInputDialog
from sensor_msgs.msg import Joy, CompressedImage
from duckietown_msgs.msg import Pose2DStamped
import numpy as np

rospy.init_node("nav_home")
rospack=rospkg.RosPack()
path=rospack.get_path('ank_ris')

steering_pub=rospy.Publisher('/lead/joy', Joy, queue_size=10, latch=True)
while steering_pub.get_num_connections() < 1 and not rospy.is_shutdown():
    pass

command=Joy()
i=0
while i<8:
    command.axes.append(0)
    i+=1

home=Pose2DStamped()
current_pose=Pose2DStamped()
return_home=False
new_home=True
lin_speed=0.5
ang_speed=1
full_circle=25
full_circle_time=2

def triggerOdom():
    command.axes[3]=0.1
    steering_pub.publish(command)
    rospy.sleep(0.3)
    command.axes[3]=0
    steering_pub.publish(command)

def stopDuck():
    global return_home
    command.axes[1]=0
    command.axes[3]=0
    steering_pub.publish(command)
    return_home=False

rospy.on_shutdown(stopDuck)

class window(QMainWindow):
    def __init__(self):
        super(window, self).__init__()
        self.setGeometry(400, 150, 1100, 400)
        self.setStyleSheet("background : grey")

        self.b1 = QtWidgets.QPushButton(self)
        self.b1.setGeometry(550, 150, 300, 50)
        self.b1.setText("Return home")
        self.b1.clicked.connect(self.b1_clicked)

        self.b2 = QtWidgets.QPushButton(self)
        self.b2.setGeometry(550, 210, 300, 50)
        self.b2.setText("Start rand drive")
        self.b2.clicked.connect(self.b2_clicked)

        self.b3 = QtWidgets.QPushButton(self)
        self.b3.setGeometry(550, 270, 300, 50)
        self.b3.setText("Stop")
        self.b3.clicked.connect(stopDuck)

        self.b4 = QtWidgets.QPushButton(self)
        self.b4.setGeometry(550, 330, 300, 50)
        self.b4.setText("Set home as current pose")
        self.b4.clicked.connect(self.b4_clicked)

        self.a1 = QtWidgets.QPushButton(self)
        self.a1.setGeometry(950, 110, 60, 60)
        self.a1.setText("^")
        self.a1.pressed.connect(self.a1_clicked)
        self.a1.released.connect(stopDuck)

        self.a2 = QtWidgets.QPushButton(self)
        self.a2.setGeometry(950, 250, 60, 60)
        self.a2.setText("v")
        self.a2.pressed.connect(self.a2_clicked)
        self.a2.released.connect(stopDuck)

        self.a3 = QtWidgets.QPushButton(self)
        self.a3.setGeometry(1020, 180, 60, 60)
        self.a3.setText(">")
        self.a3.pressed.connect(self.a3_clicked)
        self.a3.released.connect(stopDuck)

        self.a4 = QtWidgets.QPushButton(self)
        self.a4.setGeometry(880, 180, 60, 60)
        self.a4.setText("<")
        self.a4.pressed.connect(self.a4_clicked)
        self.a4.released.connect(stopDuck)

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

        self.l4 = QtWidgets.QLabel(self)
        self.l4.setGeometry(950, 180, 60, 60)
        self.l4.setPixmap(QtGui.QPixmap(path+'/imgs/logo.jpeg'))
        self.l4.setScaledContents(1)

    def b1_clicked(self):
        global return_home
        change_vector=np.array([current_pose.x-home.x, current_pose.y-home.y, current_pose.theta-home.theta])
        rospy.loginfo("change vector theta: "+str(change_vector[0]))
        if change_vector[1]<0:
            rospy.loginfo("case 1")
            rospy.loginfo("angle to home: "+str(math.degrees(np.arctan2(current_pose.y, current_pose.x))))
            rotation_correction=change_vector[2]+math.degrees(np.arctan2(current_pose.y, current_pose.x))
            command.axes[3]=-1*ang_speed
            steering_pub.publish(command)
            rospy.loginfo("rotation correction: "+str(rotation_correction))
            rospy.sleep(full_circle_time*abs(rotation_correction)/360)
            return_home=True
            command.axes[3]=0
            steering_pub.publish(command)
        else:
            rospy.loginfo("case 2")
            rospy.loginfo("angle to home: "+str(math.degrees(np.arctan2(current_pose.y, current_pose.x))))
            rotation_correction=change_vector[2]+math.degrees(np.arctan2(current_pose.y, current_pose.x))
            command.axes[3]=ang_speed
            steering_pub.publish(command)
            rospy.loginfo("rotation correction: "+str(180+rotation_correction))
            rospy.sleep(full_circle_time*abs(rotation_correction)/360)
            return_home=True
            command.axes[3]=0
            steering_pub.publish(command)
    def b2_clicked(self):
        command.axes[1]=lin_speed
        command.axes[3]=np.random.uniform(-1*lin_speed, lin_speed)
        steering_pub.publish(command)
    def b4_clicked(self):
        global new_home
        new_home=True
        triggerOdom()
    def a1_clicked(self):
        command.axes[1]=lin_speed
        steering_pub.publish(command)
    def a2_clicked(self):
        command.axes[1]=-1*lin_speed
        steering_pub.publish(command)
    def a3_clicked(self):
        command.axes[3]=-1*ang_speed
        steering_pub.publish(command)
    def a4_clicked(self):
        command.axes[3]=ang_speed
        steering_pub.publish(command)

app=QApplication(sys.argv)
win=window()
win.show()

def odom_callback(pose):
    global return_home, win, new_home, home, current_pose
    current_pose=pose
    if current_pose.theta>=0:
        current_pose.theta=(current_pose.theta%full_circle)*(360/full_circle)
    else:
        temp=(abs(current_pose.theta)%full_circle)
        current_pose.theta=(full_circle-temp)*(360/full_circle)
    win.l2.setText("Current position: "+str(round(pose.x, 2))+"x  "+str(round(pose.y, 2))+ "y  "+str(round(pose.theta, 2))+" theta")
    if return_home == True:
        command.axes[1]=lin_speed
        rospy.loginfo("dist from home: "+str(np.linalg.norm(np.array([current_pose.x, pose.y])-np.array([current_pose.x, home.y]))))
        if np.linalg.norm(np.array([current_pose.x, pose.y])-np.array([current_pose.x, home.y])) < .25:
            return_home=False
            command.axes[1]=0
        steering_pub.publish(command)
    if new_home == True:
        home=current_pose
        win.l1.setText("Home: "+str(round(home.x, 2))+"x  "+str(round(home.y, 2))+ "y  "+str(round(home.theta, 2))+" theta")
        new_home=False

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
triggerOdom()
sys.exit(app.exec_())
rospy.spin()