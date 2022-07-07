#!/usr/bin/env python
from multiprocessing import Process
from threading import Thread
import os
import rospy
from freenove_base.srv import ultrasonic_srv
from freenove_base.msg import motor_msg,servo_msg,line_tracking_msg

import cv2
import numpy as np

class chase():
    def __init__(self,TOPIC,display):
        # Init ROS
        self.TOPIC = TOPIC
        self.display = display   
        rospy.wait_for_service(self.TOPIC["ultrasonic_topic"])
        self.servo_pub = rospy.Publisher(self.TOPIC["servo_topic"], servo_msg, queue_size=3)
        self.motor_pub = rospy.Publisher(self.TOPIC["motor_topic"], motor_msg, queue_size=3)
        self.line_tracking_sub = rospy.Subscriber(self.TOPIC["line_tracking_topic"],line_tracking_msg,
                                                                self.line_tracking_callback, queue_size=3)
        self.update_moto(0,0,0,0) 
        self.update_servo(90)
        self.stage = 0
        self.LMR=0x00
        self.left = 0
        self.right = 0
        self.horizontal = 40
        self.signal = 1
        self.distance = 100

        # Init camera                 
        self.image_w = 320
        self.image_h = 240
        self.cap = cv2.VideoCapture(0)
        self.cap.set(3, self.image_w)
        self.cap.set(4, self.image_h) 
        self.safe_zone_l = int(self.image_w/2-40)
        self.safe_zone_r = int(self.image_h/2 + 40)
        self.Horizontal_center = int(self.image_w/2)

    # ultrasonic distance     
    def get_distance(self):
        #while True:
        call = rospy.ServiceProxy(self.TOPIC["ultrasonic_topic"],ultrasonic_srv)
        rusult=call(bool(1))
        #self.distance =int(rusult.distance*0.4+0.6*self.distance)
        self.distance =int(rusult.distance)
        #return rusult.distance 
        
    #  Update servo   
    def update_servo(self,horizontal,vertical=0):
        turning  = servo_msg()
        turning.horizontal = horizontal+5
        turning.vertical = vertical
        self.servo_pub.publish(turning) 

    #  Update moto       
    def update_moto(self,lu,ll,ru,rl):
        #self.get_distance()
        move = motor_msg()
        move.left_Upper_Wheel = lu
        move.left_Lower_Wheel =ll
        move.right_Upper_Wheel = ru
        move.right_Lower_Wheel= rl
        self.motor_pub.publish(move)  

    # Color filter   
    def color_plate(self,img):    
        # Image Smoothing techniques help in reducing the noise.        
        img = cv2.GaussianBlur(img,(5,5),0)
        # BGR2HSV Determine  color adjustment
        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        # Blue
        #Bmin = np.array([90, 90, 90])
        #Bmax = np.array([124, 255, 255])
        #img_bin = cv2.inRange(hsv_img,Bmin, Bmax)
        # Red 
        Rmin2 = np.array([165, 43, 46])
        Rmax2 = np.array([180, 255, 255])
        img_bin = cv2.inRange(hsv_img,Rmin2, Rmax2)
        #Use to perform erosion on the image 
        img_bin = cv2.erode(img_bin,None,iterations=3)
        _, img_bin = cv2.threshold(img_bin, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)          
        return img_bin

    def line_tracking_callback(self,msg): 
        
        self.LMR=0x00
        # Get left,mid,right sensors statement
        if msg.left==True:
            self.LMR=(self.LMR | 4)
        if msg.mid==True:
            self.LMR=(self.LMR | 2)
        if msg.right==True:
            self.LMR=(self.LMR | 1)

    def line_tracking_strategy(self):
        flag=1
        # Motors' speed and direction judgment
        if self.LMR==2:
            wheels_pose =[500,500,500,500]
        elif self.LMR==4:
            wheels_pose =[-900,-800,1200,1100]
        elif self.LMR==6:
            wheels_pose =[-1500,-1500,3000,3000]
        elif self.LMR==1:
            wheels_pose =[1100,1200,-800,-900]
        elif self.LMR==3:
            wheels_pose =[3000,3000,-1500,-1500]            
        elif self.LMR==7: 
            wheels_pose =[0,0,0,0]
        else:
            flag=0      
        # Publish only if judgment is changed           
        if flag==1:
            self.left = wheels_pose[0]
            self.right = wheels_pose[3]

    def color_tracking_strategy(self):
        success, img = self.cap.read()
        img_b=self.color_plate(img)        
        blocks,_ = cv2.findContours(img_b, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # Find target
        if len(blocks)>0 :
            print("Find target")            
            img = cv2.rectangle(img,(self.safe_zone_l, 0),
                        (self.safe_zone_r, self.image_h),
                        (0, 255, 0), 2)
            block = max(blocks, key=cv2.contourArea)
            
            rect = cv2.minAreaRect(block)
            box = cv2.boxPoints(rect)    
            area = cv2.contourArea(block)
            x, y, w, h = cv2.boundingRect(block)
            
            # Checking target area
            
            if area > 0 :# and (w/h<1.5 or h/w<1.5):
                print("area: ",area)
                cv2.drawContours(img, [np.int0(box)], -1, (0, 255, 255), 2)
                self.Horizontal_center = int(w/2+x)
                self.vertical_center = int(y + h/2)      
                img = cv2.circle(img, (self.Horizontal_center,self.vertical_center), 3, (0, 0, 0),3)            
                
                # If it is outside of safe_zone area 
                # Turn servo only
                if self.Horizontal_center >= self.safe_zone_r:
                    self.horizontal +=1
                    self.left = 0
                    self.right = 0
                    print("inside 1")
                elif self.Horizontal_center < self.safe_zone_l:
                    self.horizontal -=1
                    self.left = 0
                    self.right = 0
                    print("inside 2")
                # If it is inside of safe_zone area 
                # Turn motors 
                # Servo turn opposite direction to prevent overshoot 
                else:
                    print("inside 3")
                    if 90-self.horizontal>10:
                        self.horizontal +=1
                        self.left = -1200
                        self.right = 1200
                    elif 90-self.horizontal<-10:
                        self.horizontal -=1
                        self.left = 1200
                        self.right = -1200
                    else:
                        self.left = 800
                        self.right = 800

                # run servo
                # In limitation
                if self.horizontal<151 and self.horizontal>29:
                    self.update_servo(self.horizontal)
                # Out limitation 
                elif self.horizontal>150:
                    self.left = 800
                    self.right = -800
                    print("turn over limit")
                elif self.horizontal<30:
                    self.left = -800
                    self.right = 800
                    print("turn over limit")
                                             
        else:
            self.horizontal += self.signal*10            
            if self.horizontal>140 :
                self.horizontal = 140  
                self.signal *= -1
            elif self.horizontal<40:
                self.horizontal = 40
                self.signal *= -1
            self.update_servo(self.horizontal)
            rospy.sleep(0.2)
            # Try to find target     
            success, img = self.cap.read()
            img_b=self.color_plate(img)        
            blocks,_ = cv2.findContours(img_b, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if len(blocks)>0:
                return
        if self.display == 1:
            cv2.imshow('block_detect', img)      

    def loop(self):
        print(self.stage," In color tracking mode")
        self.color_tracking_strategy()
        print("Horizontal",self.horizontal)
        if self.horizontal <110 and self.horizontal > 80:            
            if self.display ==1:
                self.cap.release()
                cv2.destroyWindow('block_detect')          
    
        self.update_moto(self.left,self.left,self.right,self.right) 

    def end(self):
        self.update_moto(0,0,0,0)   



if __name__ == '__main__':
    print("start interface")
    rospy.init_node('color_detection', anonymous=True)
    TOPIC ={}
    TOPIC["motor_topic"] = rospy.get_param("~motor_topic",'/car/hardware/motor')
    TOPIC["servo_topic"] = rospy.get_param("~servo_topic",'/car/hardware/servo')
    TOPIC["buzzer_topic"] = rospy.get_param("~buzzer_topic",'/car/hardware/buzzer')
    TOPIC["line_tracking_topic"] = rospy.get_param("~line_tracking_topic",'/car/hardware/line_tracking')
    TOPIC["adc_topic"] = rospy.get_param("~adc_topic",'/car/hardware/adc')
    TOPIC["ultrasonic_topic"] = rospy.get_param("~ultrasonic_topic",'/car/hardware/ultrasonic')
    
    display_image = 0
    run = chase(TOPIC,display_image)
        
    while not rospy.is_shutdown(): 
        run.loop()        
        rospy.on_shutdown(run.end)                
        if cv2.waitKey(10) & 0xFF == ord('q'):
            break

