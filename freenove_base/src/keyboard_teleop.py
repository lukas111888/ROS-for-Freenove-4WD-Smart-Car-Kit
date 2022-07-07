#!/usr/bin/env python
from __future__ import print_function
import rospy
from freenove_base.srv import ultrasonic_srv
from freenove_base.msg import motor_msg
import keyboard
import termios
import tty
import sys
from select import select
import os
import signal

import threading
class teleop:
    def __init__(self,TOPIC): 
               
        self.TOPIC = TOPIC
        self.motor_pub = rospy.Publisher(self.TOPIC["motor_topic"], motor_msg, queue_size=3)
        self.speed = self.TOPIC["speed"]
        self.turn = int(self.TOPIC["turning scale"]*self.speed)        
        self.last_input = None
        self.command={}
        self.command['w'] = [self.speed,self.speed]
        self.command['s'] = [-self.speed,-self.speed]
        self.command['a'] = [-self.turn,self.turn]
        self.command['d'] = [self.turn,-self.turn]
        rospy.loginfo("Keyboard teleop control enable\n Speed:%s\tTurning scale:%s",self.speed,self.turn)
        rospy.loginfo("Focus on this terminal and use the WASD keys to drive the car.")
        rospy.loginfo("Press Ctrl-C to turn off Keyboard teleop control node.")
        rospy.loginfo("Press Ctrl-C again to terminate roscore and nodes.")

    def control(self):
        l,r = 0,0
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        timeout = 0.5
        try:
            tty.setraw(fd)
            rlist, _, _ = select([sys.stdin], [], [], timeout)
        
            if rlist:
                key = sys.stdin.read(1)
            else:
                key = '0'
            sys.stdin.flush()
        finally:
            termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, old_settings)        
        
        #print(key,':',ord(key))       
        if key in self.command:
            l,r = self.command[key]
        elif ord(key)==3:
            rospy.signal_shutdown('Teleop shut down')
        move = motor_msg()
        move.left_Upper_Wheel = l
        move.left_Lower_Wheel =l
        move.right_Upper_Wheel = r
        move.right_Lower_Wheel= r
        self.motor_pub.publish(move)
        
            



if __name__ == '__main__':
    rospy.init_node('Keyboard teleop', anonymous=False)
    
    TOPIC ={}
    TOPIC["motor_topic"] = rospy.get_param("~motor_topic",'/car/hardware/motor')
    TOPIC["speed"] = rospy.get_param("~speed",1000)
    TOPIC["turning scale"] = rospy.get_param("~turning scale",0.8)
    run = teleop(TOPIC)
    while not rospy.is_shutdown():
        run.control()
