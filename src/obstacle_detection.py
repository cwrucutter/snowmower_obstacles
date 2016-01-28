#!/usr/bin/env python
#
# The MIT License (MIT)
#
# Copyright (c) 2015 Kevin Griesser
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import rospy
from snowmower_msgs.msg import ObstacleMsg
from snowmower_msgs.msg import ObstacleArray

from sensor_msgs.msg import LaserScan
import math

class ObstacleDetection:
    def __init__(self):
        rospy.init_node('obstacle_detection')
        laserTopic = rospy.get_param('~lasertopic', '/base_scan')
        self.laserFrame = rospy.get_param('~laser_frame', 'laser')
        # Get max theta measured from center
        self.thetaMax = rospy.get_param('~maxAngle',math.pi)
        # Get max range
        self.rangeMax = rospy.get_param('~maxRange',2)
        self.obsPub = rospy.Publisher('obs/data', ObstacleArray, queue_size = 5)
        rospy.Subscriber(laserTopic, LaserScan, self.obsDetect)
    def obsDetect(self, msg):
        # Calculate the total number of measurements in lidar scan
        numScansTot = int(round((msg.angle_max-msg.angle_min)/msg.angle_increment))
        # Calculate the number we'll use based on thetaMax
        numScansUsed = int(round((2*self.thetaMax)/msg.angle_increment))
        # Don't use more scans than you have!!
        if numScansUsed > numScansTot:
            numScansUsed = numScansTot
        firstIndex = int(round((numScansTot/2-numScansUsed/2)))
        lastIndex = int(round((numScansTot/2+numScansUsed/2)))
        # Create an empty obstacle array
        obsArray = ObstacleArray()
        obsArray.obstacles = ()
        for x in xrange(firstIndex, lastIndex):
            # Check to make sure the intensity is not zero.
            if msg.intensities[x] != 0:
                # Then make sure it is in range
                if msg.ranges[x] <= self.rangeMax:
                    # If so, stick the measurement's range and angle in an
                    # ObstacleMsg and append it to the ObstacleArray
                    obsMsg = ObstacleMsg()
                    obsMsg.angle = x*msg.angle_increment+msg.angle_min
                    obsMsg.distance = msg.ranges[x]
                    obsArray.obstacles = obsArray.obstacles + (obsMsg,)
        # Then stamp it and add the frame_id
        obsArray.header.stamp = msg.header.stamp
        obsArray.header.frame_id = self.laserFrame
        # And if any obstacles were found, publish it!
        if obsArray.obstacles:
            self.obsPub.publish(obsArray)

if __name__ == "__main__":
    od = ObstacleDetection()
    rospy.spin()
