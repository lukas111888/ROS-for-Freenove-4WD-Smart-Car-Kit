#!/usr/bin/env python
import rospy
from freenove_base.srv import ultrasonic_srv
from freenove_base.msg import motor_msg,servo_msg

class run_ever:
    def __init__(self,TOPIC):        
        self.TOPIC = TOPIC
        rospy.wait_for_service(self.TOPIC["ultrasonic_service"])
        self.motor_pub = rospy.Publisher(self.TOPIC["motor_topic"], motor_msg, queue_size=1)
        self.servo_pub = rospy.Publisher(self.TOPIC["servo_topic"], servo_msg, queue_size=1)
        #self.rate = rospy.Rate(60)
        self.rang_arr = [0,0,0]
        self.pos = 30
        self.i = 0
        self.signal = 1
        self.step = 60

    def get_distance(self):
        call = rospy.ServiceProxy(self.TOPIC["ultrasonic_service"],ultrasonic_srv)
        rusult=call(bool(1))
        return rusult.distance 

    def update_moto(self,lu,ll,ru,rl):
        move = motor_msg()
        move.left_Upper_Wheel = lu
        move.left_Lower_Wheel =ll
        move.right_Upper_Wheel = ru
        move.right_Lower_Wheel= rl
        self.motor_pub.publish(move)

    def run_motor(self,rang_arr):
        if rang_arr[1] < 30 :
            self.update_moto(-1450,-1450,-1450,-1450) 
            rospy.sleep(0.1)   
            if rang_arr[0] < rang_arr[2]:
                self.update_moto(1450,1450,-1450,-1450)
            else :
                self.update_moto(-1450,-1450,1450,1450)
        elif rang_arr[0] < 30 and rang_arr[1] < 30:
            self.update_moto(1500,1500,-1500,-1500)
        elif rang_arr[2] < 30 and rang_arr[1] < 30:
            self.update_moto(-1500,-1500,1500,1500)
        elif rang_arr[0] < 20 :
            self.update_moto(2000,2000,-500,-500)
            if rang_arr[0] < 10 :
                self.update_moto(1500,1500,-1000,-1000)
        elif rang_arr[2] < 20 :
            self.update_moto(-500,-500,2000,2000)
            if rang_arr[2] < 10 :
                self.update_moto(-1500,-1500,1500,1500)
        else :
            self.update_moto(800,800,800,800)

    def run(self):        
        turning  = servo_msg()
        turning.horizontal = self.pos
        turning.vertical = 85
        self.servo_pub.publish(turning)
        start = rospy.get_rostime().to_sec()
        while rospy.get_rostime().to_sec()-start < 0.2:
            pass
        print(self.get_distance())
        self.rang_arr[self.i] = self.get_distance()
        
        self.pos +=(self.step*self.signal)
        self.i +=self.signal        
        if self.i>2 or self.i<0:
            self.run_motor(self.rang_arr)
            self.signal *=-1
            self.pos +=(self.step*self.signal)*2
            self.i +=self.signal*2
        #print(self.rang_arr)
    def end(self):
        self.update_moto(0,0,0,0)         


if __name__ == '__main__':
    rospy.init_node('run_ever', anonymous=True)
    TOPIC ={}
    TOPIC["motor_topic"] = rospy.get_param("~motor_topic",'/car/hardware/motor')
    TOPIC["servo_topic"] = rospy.get_param("~servo_topic",'/car/hardware/servo')
    TOPIC["buzzer_topic"] = rospy.get_param("~buzzer_topic",'/car/hardware/buzzer')
    TOPIC["line_tracking_topic"] = rospy.get_param("~line_tracking_topic",'/car/hardware/line_tracking')
    TOPIC["adc_topic"] = rospy.get_param("~adc_topic",'/car/hardware/adc')
    TOPIC["ultrasonic_service"] = rospy.get_param("~ultrasonic_service",'/car/hardware/ultrasonic')
    go = run_ever(TOPIC)    
    while not rospy.is_shutdown():
        # This will be invoked before actual shutdown occurs
        # prevent motor keep running
        rospy.on_shutdown(go.end)
        go.run()
    print("exit")