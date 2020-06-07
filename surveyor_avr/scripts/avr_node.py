#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import Float32
from surveyor_avr.msg import i2c_data
import smbus
import struct

class avr_node:

    def __init__(self):
        self.servo_value = 90
        self.bus = smbus.SMBus(1)
        
        self.pub = rospy.Publisher('data', i2c_data, queue_size=10)
        rospy.Subscriber("cmd_servo_angle", Float32, self.cmd_servo_angle_callback)
        
        rospy.init_node("avr_node")
        self.addr = rospy.get_param("i2c_address", 0x48)
        self.rate = rospy.Rate(rospy.get_param("i2c_rate", 30))
        

    def run(self):
        while not rospy.is_shutdown():
            self.write_i2c()
            self.pub.publish(self.read_i2c())
            self.rate.sleep()

    def write_i2c(self):
        # update avr with new values
        self.bus.write_i2c_block_data(self.addr, 12, [1, self.servo_value])

    def read_i2c(self):
        recv = self.bus.read_i2c_block_data(self.addr, 0, 16)
        recv = struct.pack("<16B", *recv)
        recv = struct.unpack("<HHHHBBBBBBH", recv)
        data = i2c_data()
        data.battery_voltage = recv[0]/64.0
        data.battery_current = recv[1]/64.0
        data.motor_current = recv[2]/64.0
        data.servo_angle = recv[8]
        return data

#    @staticmethod
    def cmd_servo_angle_callback(self, data):
        rospy.loginfo("servo got data: {}".format(data))
        if (-30/180.0*math.pi) < data.data < (30/180.0*math.pi):
            # scaling here
            self.servo_value = int(90 + data.data/math.pi * 180) 
            rospy.loginfo("new servo val: {}".format(self.servo_value))
        else:
            rospy.logerr("Invalid servo angle: {}".format(data.data))


def grab_i2c_data():
    data = i2c_data()
    data.battery_voltage = 0.0
    data.battery_current = 0.0
    data.motor_current = 0.0
    data.servo_angle = 0.0
    data.distance = 0.0
    return data
    
#def cmd_servo_angle_callback(data):
#    if (-30.0 / 180.0 * math.pi) < data.data < (30.0/180.0 * math.pi):
#        # send servo angle
#        # queue a delay for grabbing the next distance?
#        pass
#    else:
#        rospy.logerr("invalid servo angle: {}".format(data))

#def avr_node():
#    pub = rospy.Publisher('this_topic', i2c_data, queue_size=10)
#    rospy.Subscriber("cmd_servo_angle_callback", Float32, cmd_servo_angle_callback)
#    rospy.init_node('avr_node')
#    rate = rospy.Rate(1)
#    while not rospy.is_shutdown():
#        pub.publish(grab_i2c_data())
#        rate.sleep()

if __name__ == "__main__":
    try:
        node = avr_node()
        node.run()
    except rospy.ROSInterruptException:
        pass
