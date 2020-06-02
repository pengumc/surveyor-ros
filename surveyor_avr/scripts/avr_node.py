#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import Float32
from surveyor_avr.msg import i2c_data

def grab_i2c_data():
    data = i2c_data()
    data.battery_voltage = 0.0
    data.battery_current = 0.0
    data.motor_current = 0.0
    data.servo_angle = 0.0
    data.distance = 0.0
    return data
    
def cmd_servo_angle_callback(data):
    if (-30.0 / 180.0 * math.pi) < data.data < (30.0/180.0 * math.pi):
        # send servo angle
        # queue a delay for grabbing the next distance?
        pass
    else:
        rospy.logerr("invalid servo angle: {}".format(data))

def avr_node():
    pub = rospy.Publisher('this_topic', i2c_data, queue_size=10)
    rospy.Subscriber("cmd_servo_angle_callback", Float32, cmd_servo_angle_callback)
    rospy.init_node('avr_node')
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        pub.publish(grab_i2c_data())
        rate.sleep()

if __name__ == "__main__":
    try:
        avr_node()
    except rospy.ROSInterruptException:
        pass
