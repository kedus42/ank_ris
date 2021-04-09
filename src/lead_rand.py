#!/usr/bin/env python3
import rospy
import time
from sensor_msgs.msg import Joy
from std_msgs.msg import String
import numpy as np
import signal

rospy.init_node('lead_rand', disable_signals=True)
steering_pub=rospy.Publisher('/lead/joy', Joy, queue_size=30)

interval=5
lead_speed=.4
backup_speed=-.4
command=Joy()
i=0
while i<8:
    command.axes.append(0)
    i+=1
command.axes[1]=lead_speed
start=time.time()
move=True

def sigint(sig):
    command.axes[1]=0
    command.axes[3]=0
    steering_pub.publish(command)

def callback(msg):
    global command, move
    if msg=="lost":
        command.axes[1]=backup_speed
        move=False
    elif msg=="found":  
        command.axes[1]=lead_speed
        move=True

signal.signal(signal.SIGINT, sigint)
follow_sub=rospy.Subscriber('/lead/lost', String, callback=callback)

while 1 and not rospy.is_shutdown():
    if time.time()-start > interval and move:
        command.axes[3]=np.random.uniform(-.1, .1)
        start=time.time()
    steering_pub.publish(command)