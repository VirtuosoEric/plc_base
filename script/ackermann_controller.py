#!/usr/bin/env python

import rospy, math,time
import socket,threading
from geometry_msgs.msg import Twist

class PLC_Ethernet:

    def __init__(self,ip,port):
        print 'plc ip',ip
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.connect((ip, port))

    def force_set(self,register,set_value):
        if set_value:
            MESSAGE = "STS"+"20".decode("hex")+register+"20".decode("hex")+"01"+"0D".decode("hex")
        else:
            MESSAGE = "RSS"+"20".decode("hex")+register+"20".decode("hex")+"01"+"0D".decode("hex")
        self.s.send(MESSAGE)
        print MESSAGE

    def write_dm_int(self,register,value):
        MESSAGE = "WR"+"20".decode("hex")+register+".S"+"20".decode("hex")+str(value)+"0D".decode("hex")
        self.s.send(MESSAGE)
        print MESSAGE


throttle = 0
steering_angle = 0

#######PLC SETTINGS######
throttle_register = 'DM50'
steering_register = 'DM70'
plc = PLC_Ethernet('192.168.5.4',8501)
###### END ######

def twist_to_ackermann(data):
    global throttle,steering_angle
    v = data.linear.x
    omega = data.angular.z
    if omega != 0.0 and v != 0.0:
        radius = v / omega
        if v > max_speed:
            v = max_speed
        if v < -max_speed:
            v = -max_speed
        turn_radian = math.atan(wheelbase / radius)
        turn_degree = (turn_radian*180)/math.pi
        steering = turn_degree
    else:
        steering = 0

    throttle = v/max_speed*100
    steering_angle = int(steering)

    # print 'linear=',v
    # print 'steering',steering

def plc_cmd_timerCB(event):
    # print 'throttle=',throttle
    # print 'steering_angle=',steering_angle
    plc.write_dm_int(throttle_register,throttle)
    time.sleep(0.1)
    plc.write_dm_int(steering_register,steering_angle)

  

if __name__ == '__main__': 
    try:
        rospy.init_node('ackermann_controller')
        
        twist_cmd_topic = rospy.get_param('~twist_cmd_topic', '/cmd_vel') 
        wheelbase = rospy.get_param('~wheelbase', 2.5)
        max_speed = rospy.get_param('~max_speed',1)
    
        rospy.Subscriber(twist_cmd_topic, Twist, twist_to_ackermann, queue_size=1)
        plc_cmd_timer = rospy.Timer(rospy.Duration(0.3),plc_cmd_timerCB)
    
        rospy.spin()
    
    except rospy.ROSInterruptException:
        pass