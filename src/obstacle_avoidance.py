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
from math import sin, cos

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from collections import namedtuple

Obstacle = namedtuple('Obstacle', ['r', 'theta'])

class ObstacleAvoidance:
    def __init__(self):
        self.last_scan = []
        rospy.init_node('obstacle_avoidance')

        self.MAX_WIDTH = rospy.get_param('~maximum_width', 1)

        # input is lidar data
        lidarTopic = rospy.get_param('~lidar_topic', 'base_scan')
        rospy.Subscriber(lidarTopic, LaserScan, self.obsDetect)
        # and commanded velocity (pre obstaacle detection)
        inVelTopic = rospy.get_param('~in_vel_topic', 'cmd_vel_pre')
        rospy.Subscriber(inVelTopic, Twist, self.obsAvoid)
        # output is velocity command (to avoid the obstacle)
        outVelTopic = rospy.get_param('~out_vel_topic', 'cmd_vel')
        self.velPub = rospy.Publisher(outVelTopic, Twist, queue_size = 1)

    def obsDetect(self, msg: LaserScan):
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
        self.last_readings = readings
        
    def obsAvoid(self, msg: Twist):
        # Create a twist message
        vel_cmd = Twist()

        # Filter out points that are outside the travel circle
        filtered_points = self.filter_by_drive_circle(
            msg.linear.x, msg.angular.z, self.last_readings)
        # Identify the (0, 1, 2) points that need to be avoided
        left, right = self.select_closest_obstacles(v, w, filtered_points)
        # Calculate the minimum change to avoid those points

        # Choose that as the adjustment to make to cmd_vel

        # Do stuff (Currently just a pass through.)
        vel_cmd.linear.x = msg.linear.x
        vel_cmd.angular.z = msg.angular.z
        # Publish velocity command to avoid obstacle
        self.velPub.publish(vel_cmd)

    def filter_by_drive_circle(self, v, w, points):
        if (abs(w) < 0.001):
            # drive stright case
            for point in points:
                dx = point.r * sin(point.theta)
                dy = point.r * cos(point.theta)
                if abs(dx) < self.MAX_WIDTH / 2.:
                    yield point
        else:
            # curved case
            r = v / w
            for point in points:
                yield point

    def select_closest_obstacles(self, v, w, points):
        if len(points) <= 0:
            return (None, None,)
        elif len(points) == 1:
            return (points[0], points[0])
        
        for index, point in enumerate(points):
            pass
        return (points[0], points[1],)

if __name__ == "__main__":
    oa = ObstacleAvoidance()
    rospy.spin()
