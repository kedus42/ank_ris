#!/usr/bin/env python3
import rospy, sys, rospkg, cv2, math
from PyQt5 import QtWidgets, QtGui, QtCore
from PyQt5.QtWidgets import QApplication, QMainWindow, QMessageBox, QInputDialog
from sensor_msgs.msg import Joy, Image, CompressedImage
from std_msgs.msg import Float32 
#from duckietown_msgs.msg import Pose2DStamped
from ank_ris.msg import Pose2DStamped
import numpy as np
from cv_bridge import CvBridge

rospy.init_node("nav_home")
name=rospy.get_param('duckie')
rospack=rospkg.RosPack()
path=rospack.get_path('ank_ris')
bridge=CvBridge()
task="None"

#Setting up publishers
steering_pub=rospy.Publisher('/'+name+'/joy', Joy, queue_size=10, latch=True)
speed_pub=rospy.Publisher('/'+name+'/speed', Float32, queue_size=1, latch=True)
while steering_pub.get_num_connections() < 1 and not rospy.is_shutdown():
    pass

command=Joy()
i=0
while i<8:
    command.axes.append(0)
    i+=1

#Setting default speeeds and param values
home=Pose2DStamped()
current_pose=Pose2DStamped()
return_home=False
new_home=True
lin_speed=0.5
ang_speed=1
full_circle=25
full_circle_time=2
show="cam"
speed=rospy.get_param('speed')

#Trigger a quick movement on the bot to kickstart it's odom
def triggerOdom():
    command.axes[3]=0.1
    steering_pub.publish(command)
    rospy.sleep(0.3)
    command.axes[3]=0
    steering_pub.publish(command)

#Print a message to stop the duck
def stopDuck():
    global return_home
    command.axes[1]=0
    command.axes[3]=0
    steering_pub.publish(command)
    return_home=False

rospy.on_shutdown(stopDuck)

#Simple gui for user interface
class window(QMainWindow):
    def __init__(self):
        super(window, self).__init__()
        self.setGeometry(400, 150, 1200, 460)
        self.setStyleSheet("background : grey")
        self.setWindowTitle('Dashboard on '+name+' duckie')

        self.b1 = QtWidgets.QPushButton(self)
        self.b1.setGeometry(650, 150, 300, 50)
        self.b1.setText("Follow person")
        self.b1.clicked.connect(self.lb_clicked)

        self.b2 = QtWidgets.QPushButton(self)
        self.b2.setGeometry(650, 210, 300, 50)
        self.b2.setText("Start rand drive")
        self.b2.clicked.connect(self.b2_clicked)

        self.b3 = QtWidgets.QPushButton(self)
        self.b3.setGeometry(650, 270, 300, 50)
        self.b3.setText("Stop")
        self.b3.clicked.connect(stopDuck)

        self.b4 = QtWidgets.QPushButton(self)
        self.b4.setGeometry(650, 330, 300, 50)
        self.b4.setText("Follow duckie")
        self.b4.clicked.connect(self.sc_clicked)

        self.b5 = QtWidgets.QPushButton(self)
        self.b5.setGeometry(650, 390, 300, 50)
        self.b5.setText("Current speed: "+str(speed))
        self.b5.clicked.connect(self.b5_clicked)

        self.a1 = QtWidgets.QPushButton(self)
        self.a1.setGeometry(1050, 130, 60, 60)
        self.a1.setText("^")
        self.a1.pressed.connect(self.a1_clicked)
        self.a1.released.connect(stopDuck)

        self.a2 = QtWidgets.QPushButton(self)
        self.a2.setGeometry(1050, 270, 60, 60)
        self.a2.setText("v")
        self.a2.pressed.connect(self.a2_clicked)
        self.a2.released.connect(stopDuck)

        self.a3 = QtWidgets.QPushButton(self)
        self.a3.setGeometry(1120, 200, 60, 60)
        self.a3.setText(">")
        self.a3.pressed.connect(self.a3_clicked)
        self.a3.released.connect(stopDuck)

        self.a4 = QtWidgets.QPushButton(self)
        self.a4.setGeometry(980, 200, 60, 60)
        self.a4.setText("<")
        self.a4.pressed.connect(self.a4_clicked)
        self.a4.released.connect(stopDuck)

        self.l1 = QtWidgets.QLabel(self)
        self.l1.setGeometry(650, 50, 300, 20)
        self.l1.setText("Home: "+str(round(home.x, 2))+"x  "+str(round(home.y, 2))+ "y  "+str(round(home.theta, 2))+" theta")

        self.l2 = QtWidgets.QLabel(self)
        self.l2.setGeometry(650, 80, 300, 20)
        self.l2.setText("Current position: ")

        self.l5 = QtWidgets.QLabel(self)
        self.l5.setGeometry(650, 110, 300, 20)
        self.l5.setText("Current task: "+str(task))

        self.l3 = QtWidgets.QLabel(self)
        self.l3.setGeometry(10, 10, 612, 444)
        self.l3.setPixmap(QtGui.QPixmap(path+'/imgs/cam_feed_'+name+'.jpg'))
        self.l3.setScaledContents(1)

        self.l4 = QtWidgets.QLabel(self)
        self.l4.setGeometry(1050, 200, 60, 60)
        self.l4.setPixmap(QtGui.QPixmap(path+'/imgs/logo.jpeg'))
        self.l4.setScaledContents(1)
    
    #Button action for turning on human detection and following
    def lb_clicked(self):
        global show, task
        if show!="lb":
            rospy.set_param("/switch", "lb")
            show="lb"
            stopDuck()
            task="Follow person"
            self.l5.setText("Current task: "+str(task))
        else:
            rospy.set_param("/switch", "cam")
            show="cam"
            stopDuck()
            task="None"
            self.l5.setText("Current task: "+str(task))

    #Button action for turning on duckie detection and following
    def sc_clicked(self):
        global show
        if show!="sc":
            rospy.set_param("/switch", "sc")
            show="sc"
            stopDuck()
            task="Follow duck"
            self.l5.setText("Current task: "+str(task))
        else:
            rospy.set_param("/switch", "cam")
            show="cam"
            stopDuck()
            task="None"
            self.l5.setText("Current task: "+str(task))

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

    #Button action for triggering rand walk
    def b2_clicked(self):
        command.axes[1]=lin_speed
        #command.axes[3]=np.random.uniform(-1*lin_speed, lin_speed)
        command.axes[3]=0
        steering_pub.publish(command)
    def b4_clicked(self):
        global new_home
        new_home=True
        triggerOdom()
    def b5_clicked(self):
        global speed
        self.set_speed=QInputDialog(self)
        speed_changed=False
        speed, speed_changed=self.set_speed.getDouble(self, "Set speed", "Enter new speed", .1, 0, 1, 2)
        if speed_changed:
            self.b5.setText("Current speed: "+str(speed))
            speed_pub.publish(speed)

    #Button actions for printing teleop messages
    def a1_clicked(self):
        command.axes[1]=speed
        steering_pub.publish(command)
    def a2_clicked(self):
        command.axes[1]=-1*speed
        steering_pub.publish(command)
    def a3_clicked(self):
        command.axes[3]=-1*speed
        steering_pub.publish(command)
    def a4_clicked(self):
        command.axes[3]=speed
        steering_pub.publish(command)

app=QApplication(sys.argv)
win=window()
win.show()

#Odom callbakc to update gui text fields
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

#Callbakc to update gui's cam feed every time a new image is retrieved from the cameras in case default settings are on
def cam_feed(timer_info):
    global win
    if show=="cam":
        image=rospy.wait_for_message('/'+name+'/camera_node/image/compressed', CompressedImage)
        arr=np.fromstring(image.data, np.uint8)
        img=cv2.imdecode(arr, cv2.IMREAD_COLOR)#CV_LOAD_IMAGE_COLOR
        cv2.imwrite(path+'/imgs/cam_feed_'+name+'.jpg', img)
        win.l3.setPixmap(QtGui.QPixmap(path+'/imgs/cam_feed_'+name+'.jpg'))
        win.l3.setScaledContents(1)
    else:
        pass

#Callbakc to update gui's cam feed every time a new image is retrieved from the cameras in case switch is set to human detection
def lb_callback(image):
    global win
    if show=="lb":
        img = bridge.imgmsg_to_cv2(image, desired_encoding='passthrough')
        #rospy.loginfo("new detection")
        cv2.imwrite(path+'/imgs/lb_feed_'+name+'.jpg', img)
        win.l3.setPixmap(QtGui.QPixmap(path+'/imgs/lb_feed_'+name+'.jpg'))
        win.l3.setScaledContents(1)
    else:
        pass

#Callbakc to update gui's cam feed every time a new image is retrieved from the cameras in case switch is set to duckie detection
def sc_callback(image):
    global win
    if show=="sc":
        img = bridge.imgmsg_to_cv2(image, desired_encoding='passthrough')
        #rospy.loginfo("new detection")
        cv2.imwrite(path+'/imgs/sc_feed_'+name+'.jpg', img)
        win.l3.setPixmap(QtGui.QPixmap(path+'/imgs/sc_feed_'+name+'.jpg'))
        win.l3.setScaledContents(1)
    else:
        pass
    
#Setting up subscribers    
rospy.Subscriber('/'+name+'/velocity_to_pose_node/pose', Pose2DStamped, odom_callback)
rospy.Subscriber('/'+name+'/detections', Image, sc_callback)
rospy.Subscriber('/'+name+'/detections', Image, lb_callback)

#Timer to get video feed every .5 seconds
rospy.Timer(rospy.Duration(.5), cam_feed)
triggerOdom()
sys.exit(app.exec_())
rospy.spin()