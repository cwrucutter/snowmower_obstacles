import rospy
from snowmower_msgs.msg import obstacle_msg
from snowmower_msgs.msg import obstacle_array
from sensor_msgs import LaserScan

import time, os

class ObstacleDetection:
	def __init__(self):
		rospy.init_node('obstacle_detection')
		laserTopic = rospy.get_param('~lasertopic', '/base_scan')
		obsPub = rospy.Publisher('obs/data', obstacle_array, queue_size = 5)
		rospy.Subscriber(laserTopic, LaserScan, self.obsDetect)
		

	def obsDetect(self, msg):
		obsArray = obstacle_array();
		obsArray.obstacles = ();
		thetamax = 30;
		rangemax = 4;
		for x in xrange(90 - thetamax, 90 + thetamax):
			if msg.intensities[x] != 0:
				if msg.ranges[x] <= rangemax:
					obsMsg = obstacle_msg()
					obsMsg.angle = x - 90
					obsMsg.distance = msg.ranges[x]
			obsArray = obsArray + (obsMsg,)

