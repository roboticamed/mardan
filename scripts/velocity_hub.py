#!/usr/bin/python

"""
How to test:
    rostopic pub -r 10 /mardan/velocity geometry_msgs/Twist  '{linear:  {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'
"""

import mardan_msgs.srv as msrv
import rospy

from geometry_msgs.msg import Twist

# Global variables
subscriber = None
i2cService = None

i2cServiceName = 'mardan/i2c'


def subscriberCallback(msg):
    print('message received: linear: {0} angular: {1}'.format(msg.linear.x, msg.angular.x))
    
    # TODO: compute left and right motor speeds and send through the I2C service proxy
    

if __name__ == '__main__':
    
    rospy.init_node('velocity_hub')
    
    rospy.loginfo('waiting for service [{0}] to become available...'.format(i2cServiceName))
    rospy.wait_for_service(i2cServiceName)
    i2cService = rospy.ServiceProxy(i2cServiceName, msrv.i2c)
    rospy.loginfo('service [{0}] available'.format(i2cServiceName))
    
    subscriber = rospy.Subscriber('mardan/velocity', Twist, subscriberCallback)
    
    rospy.spin()
