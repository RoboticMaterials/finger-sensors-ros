#!/usr/bin/env python

import serial
import serial.tools.list_ports;
import rospy
import numpy as np
from finger_sensor_msgs.msg import FingerFAI, FingerSAI, FingerTouch

class FingerSensorNode():
    
    def __init__(self):
        num_sensors = 16
        EA = 0.3
        tol = 50

        sai = np.zeros(num_sensors)
        fai = np.zeros(num_sensors)
        fai_derivative = np.zeros(num_sensors)
        fai_derivative_prev = np.zeros(num_sensors)
        average_value = np.zeros(num_sensors)
        touch = np.full((num_sensors), False, dtype=bool)
        
        baud = 115200
        port =  [list(port)[0] for port in serial.tools.list_ports.comports() if (port[2] != 'n/a' and '16C0:0483' in port[2])][0]

        right_finger_fai_pub = rospy.Publisher('/right_finger/fai', FingerFAI, queue_size=1)
        left_finger_fai_pub = rospy.Publisher('/left_finger/fai', FingerFAI, queue_size=1)
        right_finger_sai_pub = rospy.Publisher('/right_finger/sai', FingerSAI, queue_size=1)
        left_finger_sai_pub = rospy.Publisher('/left_finger/sai', FingerSAI, queue_size=1)
        right_finger_touch_pub = rospy.Publisher('/right_finger/touch', FingerTouch, queue_size=1)
        left_finger_touch_pub = rospy.Publisher('/left_finger/touch', FingerTouch, queue_size=1)


        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            try:
                with serial.Serial(port, baud, timeout=0.1) as ser:
                    data = ser.readline().decode('utf-8')
                    data = np.fromstring(data,sep=' ',dtype=np.uint32)
                    if data.shape[0] == num_sensors:
                        sai = data
                        fai_derivative_prev = fai_derivative
                        fai_derivative = average_value - sai - fai
                        fai = average_value - sai
                        hit = fai < -50
                        release = fai > 50
                        touch[hit] = True
                        touch[release] = False
                        average_value = EA * sai + (1 - EA) * average_value
                        msg = FingerSAI()
                        self.publish_msg(sai[:8], left_finger_sai_pub, msg)
                        self.publish_msg(sai[8:], right_finger_sai_pub, msg)
                        msg = FingerFAI()
                        self.publish_msg(fai[:8], left_finger_fai_pub, msg)
                        self.publish_msg(fai[8:], right_finger_fai_pub, msg)
                        msg = FingerTouch()
                        self.publish_msg(touch[:8], left_finger_touch_pub, msg)
                        self.publish_msg(touch[8:], right_finger_touch_pub, msg)
            
            except serial.serialutil.SerialException:
                rospy.loginfo("Finger sensors disconnected") 
            rate.sleep()

    def publish_msg(self, data, publisher, msg):
        msg.sensor1 = data[0]
        msg.sensor2 = data[1]
        msg.sensor3 = data[2]
        msg.sensor4 = data[3]
        msg.sensor5 = data[4]
        msg.sensor6 = data[5]
        msg.sensor7 = data[6]
        msg.sensor8 = data[7]
        publisher.publish(msg)

if __name__ == '__main__':
    rospy.init_node('finger_sensors')
    try:
        fsn = FingerSensorNode()
    except rospy.ROSInterruptException:
        pass        
