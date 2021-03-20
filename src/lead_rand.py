#!/usr/bin/env python3
import rospy
import time
from sensor_msgs.msg import Joy
from std_msgs.msg import String
import numpy as np

rospy.init_node('lead_rand')
steering_pub=rospy.Publisher('/lead/joy', Joy, queue_size=30)

interval=5
lead_speed=.6
backup_speed=-.4
command=Joy()
i=0
while i<8:
    command.axes.append(0)
    i+=1
command.axes[1]=lead_speed
start=time.time()

def callback(msg):
    global command
    if msg=="lost":
        command.axes[1]=backup_speed
    elif msg=="found":  
        command.axes[1]=lead_speed

follow_sub=rospy.Subscriber('/lead/lost', String, callback=callback)

while 1:
    if start-time.time() > interval:
        command.axes[3]=np.random.uniform(-.5, .5)
        start=time.time()
    steering_pub.publish(command)