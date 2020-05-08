#!/usr/bin/env python
# coding:utf-8

import time
import math
import rospy
from math import pi
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class Obstacle():
    def __init__(self):
        sum=0
        i=1
        self._cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.obstacle()
      
    def get_scan(self):
        msg = rospy.wait_for_message("scan", LaserScan)
        self.scan_filter = []
        for i in range(360):
             self.scan_filter.append(msg.ranges[i])

    def obstacle(self):
        self.twist = Twist()
        while not rospy.is_shutdown():
            self.get_scan()
            if min(self.scan_filter) < 0.5:
                sum=sum+i
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0
                self._cmd_pub.publish(self.twist)
                rospy.loginfo('Stop!')
                if sum>10:
                   self.twist.linear.x = 0.16
                   self.twist.angular.z = 1.0
                   for t in range(30):
                         self._cmd_pub.publish(self.twist)
                   self.twist.linear.x = 0.16
                   self.twist.angular.z = -1.0
                   for t in range(30):
                         self._cmd_pub.publish(self.twist)
                   sum=0
            else:
                self.twist.linear.x = 0.2
                self.twist.angular.z = 0.0
                rospy.loginfo('distance of the obstacle : %f', min(self.scan_filter))
                self._cmd_pub.publish(self.twist)
                sum=0
def main():
    rospy.init_node('Driver_control')
    try:
        obstacle = Obstacle()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
