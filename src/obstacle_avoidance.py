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
from geometry_msgs.msg import Twist
from snowmower_msgs.msg import ObstacleMsg
from snowmower_msgs.msg import ObstacleArray

class ObstacleAvoidance:
    def __init__(self):
        rospy.init_node('obstacle_avoidance')
        # input is obstacle arrays from obstacle detection node
        obsTopic = rospy.get_param('~obs_topic', 'obs/data')
        rospy.Subscriber(obsTopic, ObstacleArray, self.obsAvoid)
        # output is velocity command (to avoid the obstacle)
        velTopic = rospy.get_param('~vel_topic', 'cmd_vel')
        self.velPub = rospy.Publisher(velTopic, Twist, queue_size = 1)
    def obsAvoid(self, msg):
        # Create a twist message
        vel_cmd = Twist()
        # Do stuff
        vel_cmd.linear.x = 0
        vel_cmd.angular.z = 0
        # Publish velocity command to avoid obstacle
        self.velPub.publish(vel_cmd)

if __name__ == "__main__":
    oa = ObstacleAvoidance()
    rospy.spin()
