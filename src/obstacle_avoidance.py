#!/usr/bin/env python
#
# The MIT License (MIT)
#
# Copyright (c) 2015 Kevin Griesser, 2016 Matthew A. Klein, William Baskin
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
import math
from math import sin, cos, pi

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

from collections import namedtuple

Obstacle = namedtuple('Obstacle', ['r', 'theta'])

class ObstacleAvoidance:
    def __init__(self):
        self.listOfRThetaPairs = []
        rospy.init_node('obstacle_avoidance')

        self.previousStopsignSighting = False
        self.stopTheRobotTOrF = False
        self.timeSinceStop = rospy.get_time()
        self.timeSinceStart = rospy.get_time()
        self.numStops = 0

        self.PATH_WIDTH = rospy.get_param('~path_width', 1.3)
        self.R_MAX = rospy.get_param('~r_max', 2)
        self.THETA_MIN = rospy.get_param('~theta_min', -pi/2)
        self.THETA_MAX = rospy.get_param('~theta_max', pi/2)
        self.MAX_STOPS = rospy.get_param('~max_stops', 4)
        self.MAX_STOP_TIME = rospy.get_param('~max_stop_time', 5.0)
        self.MIN_DRIVE_TIME = rospy.get_param('~min_drive_time', 3.0)
        rospy.loginfo('PATH_WIDTH = ' + str(self.PATH_WIDTH))
        rospy.loginfo('R_MAX = ' + str(self.R_MAX))

        # input is lidar data
        lidarTopic = rospy.get_param('~lidar_topic', 'base_scan')
        rospy.Subscriber(lidarTopic, LaserScan, self.detectObstacles)
        # and stop sign detector
        stopsignTopic = rospy.get_param('~stopsign_topic', 'stopsign/detected')
        rospy.Subscriber(stopsignTopic, Bool, self.detectStopsign)
        # and commanded velocity (pre obstaacle detection)
        inVelTopic = rospy.get_param('~in_vel_topic', 'cmd_vel_pre')
        rospy.Subscriber(inVelTopic, Twist, self.avoidObstacles)
        # output is velocity command (to avoid the obstacle)
        outVelTopic = rospy.get_param('~out_vel_topic', 'cmd_vel')
        self.velPub = rospy.Publisher(outVelTopic, Twist, queue_size = 1)

    def detectStopsign(self, msg):
        now = rospy.get_time()
        # If we see a stop sign and saw it before and have been moving for so many seconds and we aren't already stopped - then stop
        if (msg.data and
            True and
            now-self.timeSinceStart > self.MIN_DRIVE_TIME and
            not self.stopTheRobotTOrF and
            self.numStops < self.MAX_STOPS):
            self.stopTheRobotTOrF = True
            self.timeSinceStop = now
            self.numStops = self.numStops+1
            print('Stopping')
        # drive for at least a certain amount of time before stopping again
        if (now-self.timeSinceStop > self.MAX_STOP_TIME and
            self.stopTheRobotTOrF):
            self.stopTheRobotTOrF = False
            self.timeSinceStart = now
            print('Starting')
        # store previous sighting (needs two in a row to stop)
        self.previousStopsignSighting = msg.data

    def detectObstacles(self, msg):
        # Detect obstacles using incoming LIDAR scan
        self.last_scan = msg
        min_angle = msg.angle_min
        max_angle = msg.angle_max
        angle_increment = msg.angle_increment
        readings = []
        for index, range_ in enumerate(msg.ranges):
            angle = min_angle + index*angle_increment
            r = range_
            readings.append(Obstacle(r=r, theta=angle))
        self.listOfRThetaPairs = readings

    def avoidObstacles(self, msg):
        # Create a twist message
        vel_cmd = Twist()
        # Filter out points that are outside the Region of Interest (ROI)
        filteredListOfRThetaPairs = self.filterBySemicircleROI(
            self.listOfRThetaPairs, self.R_MAX, self.THETA_MIN, self.THETA_MAX)
        # Calculate the minimum change to avoid those points
        curvature = self.calculateCurvatureToPassObstacles(
            msg,self.PATH_WIDTH,filteredListOfRThetaPairs)
        # DEBUG
        print('Old curvature = ' + str(msg.angular.z/msg.linear.x) + ', New Curvature = ' + str(curvature))

        if (self.stopTheRobotTOrF):
            vel_cmd.linear.x = 0.0
            vel_cmd.angular.z = 0.0
        else:
            vel_cmd.linear.x = msg.linear.x
            vel_cmd.angular.z = msg.linear.x*curvature
        # Publish velocity command to avoid obstacle
        self.velPub.publish(vel_cmd)

    def filterBySemicircleROI(self, listOfRThetaPairs, rMax,
                              thetaMin, thetaMax):
        filteredListOfRThetaPairs = []
        for rThetaPair in listOfRThetaPairs:
            if rThetaPair.r < rMax and rThetaPair.theta >= thetaMin and rThetaPair.theta <= thetaMax:
                filteredListOfRThetaPairs.append(rThetaPair)
        return filteredListOfRThetaPairs

    def calculateCurvatureToPassObstacles(
            self, velocityPre, pathWidth, filteredListOfRThetaPairs):
        """Miss the obstacles but deviate from original path the least.

        Find the path that passes all obstacles to the left and the path that
        passes all obstacles to the right. Then pick the one that deviates
        from the original path the least.

        Inputs:
        velocityPre - the current velocity command, used to determine current
                      curvature.
        pathWidth - how much room is needed for path to not hit obstacle.
        filteredListOfRThetaPairs - (r,theta) coordinates of obstacles.

        Outputs:
        curvature - curvature of the path that misses all obstacles and
                    deviates from the original curvature the least.
        """
        originalCurvature = velocityPre.angular.z/velocityPre.linear.x
        # DEBUG
        print('Left')
        curvatureToPassObstaclesOnLeft = self.calculateCurvatureToPassObstaclesOnLeft(originalCurvature, pathWidth, filteredListOfRThetaPairs)
        # DEBUG
        print('Right')
        curvatureToPassObstaclesOnRight = self.calculateCurvatureToPassObstaclesOnRight(originalCurvature, pathWidth, filteredListOfRThetaPairs)
        # DEBUG
        # print('c = ' + str(originalCurvature) + ', cl = ' + str(curvatureToPassObstaclesOnLeft) + ', cr = ' + str(curvatureToPassObstaclesOnRight))
        # Return whichever curvature deviates least from the original
        if (abs(curvatureToPassObstaclesOnLeft-originalCurvature) <=
            abs(curvatureToPassObstaclesOnRight-originalCurvature)):
            return curvatureToPassObstaclesOnLeft
        else:
            return curvatureToPassObstaclesOnRight

    def calculateCurvatureToPassObstaclesOnLeft(
            self, originalCurvature, pathWidth, filteredListOfRThetaPairs):
        """Find the maximum curvature that will pass all obstacles on the left.

        The path that passes all obstacles to the left will be the path with
        the maximum curvature. If the original velocity command has an even
        higher curvature, use that instead.

        Inputs:
        originalCurvature - determines initial maxCurvature
        pathWidth - how much room is needed for path to not hit obstacle.
        filteredListOfRThetaPairs - (r,theta) coordinates of obstacles.

        Outputs:
        maxCurvature - max curvature to miss all obstacles or the original
                       curvature, whichever is greater.
        """
        maxCurvature = originalCurvature
        for rThetaPair in filteredListOfRThetaPairs:
            x = rThetaPair.r * cos(rThetaPair.theta)
            y = rThetaPair.r * sin(rThetaPair.theta)
            radiusOfCurvature = float(((x*x) + (y+float(pathWidth)/2)*(y+float(pathWidth)/2)) / float(2*(y+float(pathWidth)/2))) - float(pathWidth)/2;
            curvature = 1/radiusOfCurvature
            if (curvature > maxCurvature):
                maxCurvature = curvature
        return maxCurvature

    def calculateCurvatureToPassObstaclesOnRight(
            self, originalCurvature, pathWidth, filteredListOfRThetaPairs):
        """Find the minimum curvature that will pass all obstacles on the right

        The path that passes all obstacles to the right will be the path with
        the minimum curvature. If the original velocity command has an even
        lower curvature, use that instead.

        Inputs:
        originalCurvature - determines initial minCurvature
        pathWidth - how much room is needed for path to not hit obstacle.
        filteredListOfRThetaPairs - (r,theta) coordinates of obstacles.

        Outputs:
        minCurvature - minimum curvature to miss all obstacles or the original
                       curvature, whichever is lower.
        """
        minCurvature = originalCurvature
        for rThetaPair in filteredListOfRThetaPairs:
            x = rThetaPair.r * cos(rThetaPair.theta)
            y = rThetaPair.r * sin(rThetaPair.theta)
            radiusOfCurvature = float((x*x) + (y-float(pathWidth)/2)*(y-float(pathWidth)/2)) / float(2*(y-float(pathWidth)/2)) + float(pathWidth)/2;
            curvature = 1/radiusOfCurvature
            if (curvature < minCurvature):
                minCurvature = curvature
        return minCurvature


if __name__ == "__main__":
    oa = ObstacleAvoidance()
    rospy.spin()
