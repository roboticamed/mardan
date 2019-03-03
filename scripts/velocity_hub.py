#!/usr/bin/python

"""
How to test:
    rostopic pub -r 10 /mardan/velocity geometry_msgs/Twist  '{linear:  {x: 0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 100.0}}'
"""

import mardan_msgs.srv as msrv
import rospy
import smbus

from geometry_msgs.msg import Twist

i2cServiceName = 'mardan/i2c'

i2c_adress = 0x03

i2cBus = None


def intToByteArray(value):
    result = []
    intBytes = 2
    mask = 0xFF

    for i in range(0, intBytes):
        result.insert(0, value & mask)
        value >>= 8

    return result


def subscriberCallback(msg):
    global i2cBus
    print('message received: linear: {0} angular: {1}'.format(msg.linear.x, msg.angular.z))
    
    var_r = intToByteArray(int(msg.linear.x + msg.angular.z))
    var_l = intToByteArray(int(msg.linear.x - msg.angular.z))
    
    data = var_r + var_l
    print('data: {0}'.format(str(data)))
    
    i2cBus.write_i2c_block_data(i2c_adress, 0, data)
    

if __name__ == '__main__':
    
    i2cBus = smbus.SMBus(1)
    
    rospy.init_node('velocity_hub')
    
    subscriber = rospy.Subscriber('mardan/velocity', Twist, subscriberCallback)
    
    rospy.spin()
