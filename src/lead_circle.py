#!/usr/bin/env python3
import rospy
import time
from sensor_msgs.msg import Joy
from std_msgs.msg import String
import numpy as np

rospy.init_node('lead_circle')
steering_pub=rospy.Publisher('/lead/joy', Joy, queue_size=30)

lead_speed=.6
backup_speed=-.4
steer_at=.5
move=True
command=Joy()
i=0
while i<8:
    command.axes.append(0)
    i+=1
command.axes[1]=lead_speed

def callback(msg):
    global command, move
    if msg=="lost":
        command.axes[1]=backup_speed
        move=False
    elif msg=="found":  
        command.axes[1]=lead_speed
        move=True

follow_sub=rospy.Subscriber('/lead/lost', String, callback=callback)

while 1:
    if move:
        command.axes[3]=steer_at
    else:
        command.axes[3]=0
    steering_pub.publish(command)